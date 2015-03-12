#!/usr/bin/python
import errno
import fcntl
import itertools
import json
import logging
import optparse
import os
import serial
import signal
import socket
import subprocess
import sys
import time
import traceback

import trollius as asyncio
from trollius import From, Return

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

from vui_helpers import wrap_event, FCMD, CriticalTask
import vui_helpers

# How often to poll servo status, in absense of other commands
# (polling only starts once first remote command is received)
SERVO_SEND_INTERVAL = 0.5

# Turret ranges: (min, max) pairs in degrees
TURRET_RANGE_X = (-120, 120)
TURRET_RANGE_Y = (-35, 30)

# How fast should turret servoes move into position
TURRET_POSE_TIME = 0.25

# Servo IDs of turret servoes
TURRET_SERVO_X = 12
TURRET_SERVO_Y = 13

# How long to fastpoll turret for after it stopped moving
# (turret is always fastpolled while it is moving)
TURRET_FASTPOLL_TIME = 1.0

# TODO mafanasyev: add overload controls to detect gun hitting
#                  limit
TURRET_SERVO_CONFIG = {
    'dead_zone': 0,     # when to shut off the motor
    # We set larger 'inpos_margin' and rely on stop_thresh
    # to fire when we are alsmot at a target and no progress is being made
    'inpos_margin': 2,  # 'inposition' sensitivity
    'stop_thresh': 1,   # 'moving' sensitivity
    'stop_time': 100,   # 'moving' timeout, in 11mS units
    # 'acc_ratio': 50,  # triangular drive profile

    # default P/I/D is 254/0/6500
    # stable but slow: 200/1000/1000
    'pos_kp': 200, 'pos_ki': 5000, 'pos_kd': 1000,
    }
# Modifications for 'Y' servo
TURRET_SERVO_CONFIG_Y = {
    'pos_kp': 300, 'pos_ki': 5000, 'pos_kd': 1000,
}

# How long to run agitator for after each shot
AUTO_AGITATOR_TIME = 2.0

# Which servo IDs to poll for status (must be non-empty)
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
        # When >now, turret servoes will be polled at a higher rate
        self.turret_fastpoll_until = 0
        # Last commanded turret position.
        self.turret_last_command = None
        # Time when last_command changed
        self.turret_last_command_time = 0

        self.gait_commanded_nonidle = False
        # When >now, agitator will be enabled when in auto mode
        self.auto_agitator_expires = 0

        # Last processed fire command
        self.last_fire_cmd = None

        # Iterator which returns next servo to poll
        self.next_servo_to_poll = itertools.cycle(SERVO_IDS_TO_POLL)
        self.servo_poll_count = 0

        # Normally we refresh state periodically. Set this flag to force
        # re-sending the data.
        self.servo_send_now = asyncio.Event()
        CriticalTask(self._send_servo_commands())

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
            "agitator_on": 0,
            "turret_position": [None, None],  # actual turret position
            # Empty string if motion is done, else reason
            "turret_inmotion": None,
            "shots_fired": 0,
            }

        CriticalTask(self._send_status_packets())
        self.status_send_now = asyncio.Event()

        self.mech_driver_started = False
        if self.opts.no_mech:
            self.mech_driver = None
        else:
            self.mech_driver = gait_driver.MechDriver(opts)
            CriticalTask(self.mech_driver.connect_servo(),
                         exit_ok=True)

        self.video_addr = None
        self.video_proc = None
        gobject.timeout_add(int(self.TIMEOUT * 1000),
                            wrap_event(self._on_timeout))
        gobject.io_add_watch(self.sock.fileno(),
                             gobject.IO_IN | gobject.IO_ERR | gobject.IO_HUP,
                             wrap_event(self._on_readable))

    def _on_timeout(self):
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
            yield From(asyncio.sleep(0.05))

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
        if servo_id in [TURRET_SERVO_X, TURRET_SERVO_Y]:
            status_list, position = yield From(
                self.mech_driver.servo.get_clear_status(
                    servo_id, return_pos=True))
        else:
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

        if (servo_id == TURRET_SERVO_X) and (position is not None):
            self.status_packet['turret_position'][0] = -position
        elif (servo_id == TURRET_SERVO_Y) and (position is not None):
            self.status_packet['turret_position'][1] = position

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

        now = time.time()
        agitator_on = (data['agitator_mode'] == 2)
        if data['agitator_mode'] == 1 and (self.auto_agitator_expires > now):
            agitator_on = 1

        self.status_packet['agitator_on'] = agitator_on
        # re-sent magic servo command. Unlike real servo,
        # it will timeout and turn off all LEDs if there were no
        # commands for a while.
        yield From(servo.mjmech_set_outputs(
                laser=data['laser_on'],
                green=data['green_led_on'],
                # LED_BLUE is an actual blue LED on the board. Flash it
                # as we receive the packets.
                blue=(self.recent_packets % 2),
                agitator=(data['agitator_pwm'] if agitator_on else 0)
                ))

        poll_servo_id = self.next_servo_to_poll.next()
        now = time.time()   # update now -- we sent a command

        turret_inmotion = []
        new_command = False
        # check for new turret command
        if data.get('turret') != self.turret_last_command:
            self.turret_last_command = data['turret']
            self.turret_last_command_time = now
            self.turret_fastpoll_until = now + TURRET_FASTPOLL_TIME
            new_command = True

        # Send command if needed -- when changed or periodically
        if (new_command or (self.servo_poll_count % 113 == 0)
            ) and data.get('turret'):
            if not self.turret_servoes_ready:
                self.turret_servoes_ready = True
                sid_list = [TURRET_SERVO_X, TURRET_SERVO_Y]
                # Enable power
                yield From(servo.enable_power(selector.POWER_ENABLE, sid_list))
                # Configure servo
                config = dict(TURRET_SERVO_CONFIG)
                yield From(servo.configure_servo(config, [TURRET_SERVO_X]))
                config.update(TURRET_SERVO_CONFIG_Y)
                yield From(servo.configure_servo(config, [TURRET_SERVO_Y]))

            servo_x, servo_y = data['turret']
            send_x = min(TURRET_RANGE_X[1],
                         max(TURRET_RANGE_X[0], -servo_x))
            send_y = min(TURRET_RANGE_Y[1],
                         max(TURRET_RANGE_Y[0], servo_y))
            yield From(servo.set_pose({TURRET_SERVO_X: send_x,
                                       TURRET_SERVO_Y: send_y},
                                      pose_time=TURRET_POSE_TIME))

        # Poll servoes if we are in fastpoll mode
        if self.turret_fastpoll_until > now:
            for axis, sid in (('x', TURRET_SERVO_X),
                              ('y', TURRET_SERVO_Y)):
                status = yield From(self._poll_servo_status(sid))
                if 'moving' in status:
                    turret_inmotion.append(axis + '.moving')
                elif 'inposition' not in status:
                    # 'inposition' seems to be always false when moving is true
                    turret_inmotion.append(axis + '.ninpos')
                if sid == poll_servo_id:
                    # Do not poll for turret servos again
                    poll_servo_id = self.next_servo_to_poll.next()

            # TODO mafanasyev: also add requirement to be in steady state
            # for X samples?

        if new_command and not turret_inmotion:
            turret_inmotion.append('cmd')

        # Update new inmpotion flags
        inmotion_str = ','.join(turret_inmotion)
        if self.status_packet['turret_inmotion'] != inmotion_str:
            self.status_packet['turret_inmotion'] = inmotion_str
            dt = now - self.turret_last_command_time
            if inmotion_str:
                self.logger.debug(
                    'Turret in motion: %s (dt %.3f)', inmotion_str, dt)
            else:
                self.logger.debug(
                    'Turret no longer in motion (dt %.3f)', dt)

        if turret_inmotion:
            # Keep fastpolling while we are moving, and then some
            self.turret_fastpoll_until = now + TURRET_FASTPOLL_TIME

        # poll other servos (one at a time)
        self.servo_poll_count += 1
        special_step = 0
        if (poll_servo_id != 99) and (
            'offline' not in self.last_servo_status.get(poll_servo_id, [])):
            # Note that mod-factor should be relatively prime to number
            # of servoes
            special_step = (self.servo_poll_count % 103)
        if special_step == 10:
            voltage_dict = yield From(servo.get_voltage([poll_servo_id]))
            self.status_packet['servo_voltage'][poll_servo_id] = \
                voltage_dict.get(poll_servo_id, None)
        elif special_step == 18:
            temp_dict = yield From(servo.get_temperature([poll_servo_id]))
            self.status_packet['servo_temp'][poll_servo_id] = \
                temp_dict.get(poll_servo_id, None)
        else:
            yield From(self._poll_servo_status(poll_servo_id))

        # Process firing
        fire_cmd = data['fire_cmd']
        if fire_cmd is None and self.last_fire_cmd is not None:
            self.last_fire_cmd = None
            # Stop firing motor
            yield From(servo.mjmech_fire(fire_time=0, fire_pwm=0))
        elif fire_cmd == self.last_fire_cmd:
            # No new command -- current one is processed
            pass
        elif data['fire_cmd_deadline'] < now:
            # Command expired
            self.last_fire_cmd = fire_cmd
            dt = now - data['fire_cmd_deadline']
            self.logger.warn('Ignoring old fire_cmd %r: %.3f sec old',
                             fire_cmd, dt)
        else:
            command, seq = fire_cmd
            # Start agitator early, but only for every Nth shot
            # (The reason for that is if there is a BB stuck in the agitator,
            # you need to fire with agitator DISABLED in order to dislodge it)
            if (seq % 2) == 0:
                self.auto_agitator_expires = now + AUTO_AGITATOR_TIME

            if FCMD._is_inpos(command) and turret_inmotion:
                # We are not in position yet. Do not fire.
                pass
                # Do not touch last_fire_cmd so we keep retrying the test
            else:
                # Firetime!
                # We can do few shots at once, but no more than 25.5 sec.
                self.last_fire_cmd = fire_cmd
                command = fire_cmd[0]
                if command == FCMD.cont:
                    duration = 0.5  # bigger than poll interval
                    # TODO mafanasyev: implement
                    numshots = 1
                else:
                    numshots = FCMD._numshots(command)
                    duration = data['fire_duration'] * numshots
                self.logger.debug('Bang bang %.2f sec!!', duration)

                # TODO mafanasyev: do not add it if previous shot is not
                # yet complete.
                self.status_packet["shots_fired"] += numshots

                yield From(servo.mjmech_fire(
                        fire_time=duration, fire_pwm=data['fire_motor_pwm']))

        if data.get('gait'):
            # We got a gait command. Start the mech driver.
            if self.mech_driver and not self.mech_driver_started:
                self.mech_driver_started = True
                # Gait engine started, but no motion
                self.status_packet["last_motion_time"] = None
                self.logger.debug('Gait engine started')
                CriticalTask(self.mech_driver.run())

            gait = data['gait']
            if gait['type'] == 'idle':
                # TODO mafanasyev: update last_motion_time with the time
                # motion actually ends.
                self.mech_driver.set_idle()
                if self.gait_commanded_nonidle:
                    self.logger.debug('No longer commanding gait')
                    self.gait_commanded_nonidle = False
            elif gait['type'] == 'ripple':
                self.status_packet["last_motion_time"] = time.time()

                command = self.mech_driver.create_default_command()

                # Now apply any values we received from the client.
                for key, value in gait.iteritems():
                    if key != 'type':
                        setattr(command, key, value)

                if not self.gait_commanded_nonidle:
                    self.logger.debug('Commanding nonidle gait %r' % gait)
                    self.gait_commanded_nonidle = True
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
        plogger = logging.getLogger('vsender')
        pid, _1, fd_out, fd_err = gobject.spawn_async(
            ['./send-video.sh', str(addr[0]), str(addr[1])],
            flags=gobject.SPAWN_DO_NOT_REAP_CHILD,
            standard_output=True, standard_error=True)
        self.video_proc = pid
        plogger.debug('Started video process, PID %r' % pid)
        CriticalTask(vui_helpers.dump_lines_from_fd(
                fd_out, plogger.debug), exit_ok=True)
        CriticalTask(vui_helpers.dump_lines_from_fd(
                fd_err, plogger.getChild('stderr').debug),
                     exit_ok=True)
        gobject.child_watch_add(pid, self._video_process_died)

    def _video_process_died(self, pid, condition):
        if pid == self.video_proc:
            self.logger.info('Video process %d died: %r' % (pid, condition))
            self.video_proc = None
        else:
            self.logger.error('Unknown video process %d died: %r' %
                              (pid, condition))

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
    proc = subprocess.Popen(
        "killall -v gst-launch-1.0",
        shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT);
    outmsg, _ = proc.communicate()
    for line in outmsg.split('\n'):
        line = line.strip()
        if line != '': logging.debug('Cleanup says: ' + line)
    logging.debug('Cleanup complete, result %r' % proc.returncode)

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
