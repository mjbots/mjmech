#!/usr/bin/python
import sys
import os
import traceback
import logging
import socket
import optparse
import json
import errno
import time
import signal
import serial
import fcntl
import itertools

import trollius as asyncio
from trollius import Task, From, Return

from gi.repository import GObject
gobject = GObject

sys.path.append(os.path.join(os.path.dirname(__file__), '../../legtool'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'legtool'))

import gait_driver
import gbulb
import legtool
import legtool.servo.selector as selector

from vui_helpers import wrap_event, FCMD
import vui_helpers

# How often to poll servo status, in absense of other commands
# (polling only starts once first remote command is received)
SERVO_SEND_INTERVAL = 0.5

# Turret ranges: (min, max) pairs in degrees
TURRET_RANGE_X = (-90, 90)
TURRET_RANGE_Y = (-90, 90)

# Servo IDs of turret servoes
TURRET_SERVO_X = 12
TURRET_SERVO_Y = 13

# Which servo IDs to poll for status
# (set to false value to disable mechanism)
SERVO_IDS_TO_POLL = [1, 3, 5, 7, TURRET_SERVO_X, TURRET_SERVO_Y, 99]

_start_time = time.time()

class ControlInterface(object):
    PORT = 13356

    TIMEOUT = 1.0

    def __init__(self, opts, logsaver):
        self.logger = logging.getLogger('control')

        self.logsaver = logsaver
        self.opts = opts

        self.sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setblocking(0)
        self.logger.debug('Binding to port %d' % self.PORT)
        # Bind to port early (before opening serial device).
        # This ensures only once instance is active.
        self.sock.bind(('', self.PORT))

        self.src_addr = None
        # Packets since timeout expiration
        self.recent_packets = 0

        # timeout intervals with no packets (start with very high value to
        # suppress warnings until client is connected)
        self.no_packet_intervals = 10000

        # Last packet received from the network (set to None after timeout)
        self.net_packet = None

        # Last packet applied to servoes (set to None if restart detected or
        #  net_packet is None)
        self.servo_packet = None

        # Per-servo status (address->tuple of strings)
        # (this excludes some volatile status bits)
        self.last_servo_status = dict()
        self.turret_servoes_ready = False

        # Last processed fire command
        self.last_fire_cmd = None

        # Iterator which returns next servo to poll
        if SERVO_IDS_TO_POLL:
            self.next_servo_to_poll = itertools.cycle(SERVO_IDS_TO_POLL)
        else:
            self.next_servo_to_poll = None
        self.servo_poll_count = 0

        # Normally we refresh state periodically. Set this flag to force
        # re-sending the data.
        self.servo_send_now = asyncio.Event()
        self.servo_send_task = Task(self._send_servo_commands())

        # We send status every time this event is set (but in a rate-limited
        # way)
        self.status_send_now = asyncio.Event()

        # The contents of the status packet to send.
        self.status_packet = {
            "start_time": _start_time,
            "seq": 0,
            "servo_status": dict(),
            "servo_voltage": dict(),
            "servo_temp": dict(),
            }

        self.status_send_task = Task(self._send_status_packets())
        self.status_send_now = asyncio.Event()

        self.mech_driver_started = False
        if self.opts.no_mech:
            self.mech_driver = None
        else:
            self.mech_driver = gait_driver.MechDriver(opts)
            Task(self.mech_driver.connect_servo())

        self.video_addr = None
        self.video_proc = None
        gobject.timeout_add(int(self.TIMEOUT * 1000),
                            wrap_event(self._on_timeout))
        gobject.io_add_watch(self.sock.fileno(),
                             gobject.IO_IN | gobject.IO_ERR | gobject.IO_HUP,
                             wrap_event(self._on_readable))

    def _on_timeout(self):
        if self.servo_send_task and self.servo_send_task.done():
            # super ugly hack to get real exception: lose the object
            # (if we call result(), then stack trace is lost; otherwise
            # default exception handler is activated which prints proper
            # traceback)
            self.servo_send_task = None
            raise Exception("Servo send task exited")

        if self.status_send_task and self.status_send_task.done():
            # ditto
            self.status_send_task = None
            raise Exception("Status send task exited")


        if self.recent_packets:
            #self.logger.debug('Remote peer %r sent %d packet(s)' % (
            #        self.src_addr, self.recent_packets))
            self.recent_packets = 0
            self.no_packet_intervals = 0
        elif self.no_packet_intervals < 3:
            self.no_packet_intervals += 1
            self.logger.info(
                'No data from peer %r (%d times). It possibly went away',
                self.src_addr, self.no_packet_intervals)
        elif self.src_addr:
            self.logger.debug('Remote peer %r went away' % (self.src_addr, ))
            self._set_src_addr(None)
        else:
            self._set_video_dest(None)
        return True

    def _on_readable(self, source, cond):
        while True:
            try:
                pkt, addr = self.sock.recvfrom(65535)
            except socket.error as e:
                if e.errno != errno.EAGAIN:
                    raise
                break
            if addr != self.src_addr:
                self._set_src_addr(addr)
            self._handle_packet(pkt)
        return True

    def _set_src_addr(self, addr):
        if self.src_addr is not None:
            self.logger.info('Remote peer address changed: %r->%r' %
                             (self.src_addr, addr))
        else:
            self.logger.debug('Remote peer address is now %r' % (addr, ))
        self.src_addr = addr
        if addr is None:
            self.net_packet = None
            self.servo_packet = None
        self._set_video_dest(None)

    def _handle_packet(self, pkt_bin):
        self.recent_packets += 1
        pkt = json.loads(pkt_bin)

        if self.net_packet is None:
            pass
        elif self.net_packet['boot_time'] != pkt['boot_time']:
            self.logger.warn('Client restart detected, new seq %r',
                              pkt['seq'])
            # Wipe out all internal state
            # (we will re-start with the next packet)
            self._set_src_addr(None)
            return
        elif (self.net_packet['seq'] - 100) \
                <= pkt['seq'] <= self.net_packet['seq']:
            self.logger.info(
                'Dropping out-of-order packet: last seq %r, got %r',
                self.net_packet['seq'], pkt['seq'])
            return
        elif self.net_packet['seq'] != (pkt['seq'] - 1):
            self.logger.info('Seq number jump: %r->%r',
                             self.net_packet['seq'], pkt['seq'])

        vport = pkt.get('video_port', 0)
        if not vport:
            self._set_video_dest(None)
        else:
            self._set_video_dest((self.src_addr[0], vport))

        self.net_packet = pkt
        # Wake up servo sender so it processes new state
        self.servo_send_now.set()

    @asyncio.coroutine
    @wrap_event
    def _send_status_packets(self):
        while True:
            yield From(self.status_send_now.wait())
            self.status_send_now.clear()

            self.status_packet["seq"] += 1
            self.status_packet["srv_time"] = time.time()
            if self.src_addr:
                # Add most recent log messages if requested
                if self.net_packet and self.net_packet.get('logs_from'):
                    ts_min = self.net_packet['logs_from']
                    # Discard already-seen messages
                    while (self.logsaver.data and
                           self.logsaver.data[0][0] <= ts_min):
                        self.logsaver.data.pop(0)
                    dest = self.status_packet['logs_data'] = list()
                    # Insert up to new messages (but not all, to limit the size)
                    dest += self.logsaver.data[:16]
                else:
                    # Do not send logs_data if logs_from is missing.
                    self.status_packet.pop('logs_data', None)

                # send packet
                payload = json.dumps(self.status_packet)
                self.sock.sendto(payload, self.src_addr)

            # ratelimit
            yield From(asyncio.sleep(0.1))

    @asyncio.coroutine
    def _send_servo_commands(self):
        servo_started_up = False

        while True:
            if ((not servo_started_up)
                and self.mech_driver
                and self.mech_driver.servo):
                servo_started_up = True
                # Reset all servos on startup, as they may have had an pending
                # error due to power glitch or something similar.
                self.logger.info('Rebooting servos on startup')
                yield From(self.mech_driver.servo.reboot())

            if not self.servo_send_now.is_set():
                # Make sure we wake up periodically
                asyncio.get_event_loop().call_later(
                    SERVO_SEND_INTERVAL, self.servo_send_now.set)
            yield From(self.servo_send_now.wait())
            self.servo_send_now.clear()

            new_pkt = self.net_packet
            old_pkt = self.servo_packet
            if new_pkt is not None:
                yield From(self._send_servo_commands_once(
                        new_pkt, old_pkt))
            self.servo_packet = new_pkt

            # send any status updates
            self.status_send_now.set()

    @asyncio.coroutine
    def _poll_servo_status(self, servo_id):
        """Poll servo's status. Convert result to string, update
        state vars, return new result.
        """
        status_list = yield From(
            self.mech_driver.servo.get_clear_status(servo_id))

        status_list_clean = tuple(
            x for x in status_list if x not in ['moving', 'inposition'])
        if status_list_clean \
                != self.last_servo_status.get(servo_id, ('motor_off', )):
            self.logger.debug(
                'Servo status for %r: %s' % (
                    servo_id, ','.join(status_list_clean) or 'ready'))
        self.last_servo_status[servo_id] = status_list_clean

        status_str = ','.join(status_list) or 'idle'
        self.status_packet['servo_status'][servo_id] = status_str

        raise Return(status_list)

    @asyncio.coroutine
    def _send_servo_commands_once(self, data, olddata):
        if not self.mech_driver:
            return
        servo = self.mech_driver.servo
        if servo is None:
            self.logger.debug('Not sending commands -- no servo yet')
            raise Return()

        if olddata is None:
            olddata = dict()   # so one can use 'get'

        # re-sent magic servo command. Unlike real servo,
        # it will timeout and turn off all LEDs if there were no
        # commands for a while.
        yield From(servo.mjmech_set_outputs(
                laser=data['laser_on'],
                green=data['green_led_on'],
                # LED_BLUE is an actual blue LED on the board. Flash it
                # as we receive the packets.
                blue=(self.recent_packets % 2),
                agitator=(data['agitator_pwm'] if data['agitator_on'] else 0)
                ))

        # poll servos (one at a time)
        if self.next_servo_to_poll is not None:
            servo_id = self.next_servo_to_poll.next()
            self.servo_poll_count += 1
            special_step = 0
            if (servo_id != 99) and \
                    'offline' not in self.last_servo_status.get(servo_id, []):
                # Note that mod-factor should be relatively prime to number
                # of servoes
                special_step = (self.servo_poll_count % 103)

            if special_step == 10:
                voltage_dict = yield From(servo.get_voltage([servo_id]))
                self.status_packet['servo_voltage'][servo_id] = \
                    voltage_dict.get(servo_id, None)
            elif special_step == 18:
                temp_dict = yield From(servo.get_temperature([servo_id]))
                self.status_packet['servo_temp'][servo_id] = \
                    temp_dict.get(servo_id, None)
            else:
                yield From(self._poll_servo_status(servo_id))

        if data.get('turret'):
            if not self.turret_servoes_ready:
                self.turret_servoes_ready = True
                sid_list = [TURRET_SERVO_X, TURRET_SERVO_Y]
                # Enable power
                yield From(servo.enable_power(selector.POWER_ENABLE, sid_list))
                # Configure deadband
                yield From(servo.configure_servo(
                        {'dead_zone': 1},
                        sid_list))

            servo_x, servo_y = data['turret']
            send_x = min(TURRET_RANGE_X[1],
                         max(TURRET_RANGE_X[0], -servo_x))
            send_y = min(TURRET_RANGE_Y[1],
                         max(TURRET_RANGE_Y[0], servo_y))
            yield From(servo.set_pose({TURRET_SERVO_X: send_x,
                                       TURRET_SERVO_Y: send_y}))

        fire_cmd = data['fire_cmd']
        if fire_cmd is None and self.last_fire_cmd is not None:
            self.last_fire_cmd = None
            # Stop firing motor
            yield From(servo.mjmech_fire(fire_time=0, fire_pwm=0))
        elif fire_cmd == self.last_fire_cmd:
            # No new command -- current one is processed
            pass
        elif data['fire_cmd_deadline'] < time.time():
            # Command expired
            self.last_fire_cmd = fire_cmd
            dt = time.time() - data['fire_cmd_deadline']
            self.logger.warn('Ignoring old fire_cmd %r: %.3f sec old',
                             fire_cmd, dt)
        else:
            command = fire_cmd[0]
            suppress = []
            if FCMD._is_inpos(command):
                # Need to check if turret servoes are inposition
                # For that, immediately poll them
                for sid in (TURRET_SERVO_X, TURRET_SERVO_Y):
                    status = yield From(self._poll_servo_status(sid))
                    if 'inposition' not in status:
                        suppress.append(sid)

            if suppress:
                self.logger.debug('Waiting for firing servoes to settle (%r)',
                                  suppress)
                # Do not touch last_fire_cmd so we keep retrying the test
            else:
                # Firetime!
                # We can do few shots at once, but no more than 25.5 sec.
                self.last_fire_cmd = fire_cmd
                command = fire_cmd[0]
                if command == FCMD.cont:
                    duration = 0.5  # bigger than poll interval
                else:
                    duration = data['fire_duration'] * FCMD._numshots(command)
                self.logger.debug('Bang bang %.2f sec!!', duration)
                yield From(servo.mjmech_fire(
                        fire_time=duration, fire_pwm=data['fire_motor_pwm']))

        if data.get('gait'):
            # We got a gait command. Start the mech driver.
            if self.mech_driver and not self.mech_driver_started:
                self.mech_driver_started = True
                # Gait engine started, but no motion
                self.status_packet["last_motion_time"] = None
                Task(self.mech_driver.run())

            gait = data['gait']
            if gait['type'] == 'idle':
                # TODO mafanasyev: update last_motion_time with the time
                # motion actually ends.
                self.mech_driver.set_idle()
            elif gait['type'] == 'ripple':
                self.status_packet["last_motion_time"] = time.time()

                command = self.mech_driver.create_default_command()

                # Now apply any values we received from the client.
                for key, value in gait.iteritems():
                    if key != 'type':
                        setattr(command, key, value)

                self.mech_driver.set_command(command)
            else:
                assert False, 'Invalid gait type %r' % gait['type']


    def _set_video_dest(self, addr):
        """This function should be called periodically -- it might not
        finish on the first run.
        """
        if addr == self.video_addr:
            return
        if self.video_proc:
            self.logger.info('Killing old video process')
            os.kill(self.video_proc, signal.SIGTERM)
            # return for now. We did not update video_addr, so we will
            # keep re-entering unless process dies.
            return
        self.video_addr = addr
        if addr is None:
            return
        self.logger.info('Sending video to %r' % (addr, ))
        pid, _1, _2, _3 = gobject.spawn_async(
            ['./send-video.sh', str(addr[0]), str(addr[1])],
            flags=gobject.SPAWN_DO_NOT_REAP_CHILD)
        self.video_proc = pid
        self.logger.debug('Started video process, PID %r' % pid)
        gobject.child_watch_add(pid, self._process_died)

    def _process_died(self, pid, condition):
        if pid == self.video_proc:
            self.logger.info('Video process %d died: %r' % (pid, condition))
            self.video_proc = None
        else:
            self.logger.info('Unknown process %d died: %r' % (pid, condition))

        # TODO mafanasyev: how to close PID properly?
        #gobject.g_spawn_close_pid(pid)

def main(opts):
    vui_helpers.asyncio_misc_init()

    vui_helpers.logging_init(verbose=True)

    logsaver = vui_helpers.MemoryLoggingHandler(install=True)

    if opts.check:
        logging.info('Check passed')
        return
    # cleanup hack
    os.system("killall -v gst-launch-1.0")

    cif = ControlInterface(opts, logsaver=logsaver)
    logging.debug('Running')

    asyncio.get_event_loop().run_forever()

if __name__ == '__main__':
    parser = optparse.OptionParser()
    parser.add_option('--check', action='store_true',
                      help='Exit immediately')
    parser.add_option('--no-mech', action='store_true',
                      help='Do not try and control mech')

    gait_driver.MechDriver.add_options(parser)

    opts, args = parser.parse_args()
    if args:
        parser.error('No args accepted')
    main(opts)
