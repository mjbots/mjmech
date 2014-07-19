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

import trollius as asyncio
from trollius import Task

from gi.repository import GObject
gobject = GObject

sys.path.append(os.path.join(os.path.dirname(__file__), '../../legtool'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../'))

import gait_driver
import gbulb
import legtool

g_main_loop = gobject.MainLoop()

def wrap_event(callback, *args1, **kwargs1):
    """Wrap event callback so the app exit if it crashes"""
    def wrapped(*args2, **kwargs2):
        try:
            kwargs = kwargs1.copy()
            kwargs.update(**kwargs2)
            return callback(*(args1 + args2), **kwargs)
        except BaseException as e:
            logging.error("Callback %r crashed:", callback)
            logging.error(" %s %s" % (e.__class__.__name__, e))
            g_main_loop.quit()
            raise
    return wrapped

class UdpAnnouncer(object):
    PORT = 13355
    INTERVAL = 0.5

    def __init__(self):
        self.logger = logging.getLogger('announcer')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setblocking(0)
        #self.logger.info('Binding to port %d' % self.PORT)
        #self.sock.bind(('', self.PORT))
        self.logger.info('Will broadcast on port %d every %.2f seconds' % 
                         (self.PORT, self.INTERVAL))
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.dest = ('255.255.255.255', self.PORT)
        self.seq = 0
        self.start_time = time.time()
        gobject.timeout_add(int(self.INTERVAL * 1000),
                            wrap_event(self._broadcast))
    def _broadcast(self):
        #self.logger.debug('Broadcasting anouncement')
        self.seq += 1
        msg = json.dumps({'type': 'announce', 
                          'seq': self.seq,
                          'cport': ControlInterface.PORT,
                          'start_time': self.start_time,
                          'host': os.uname()[1]})
        self.sock.sendto(msg + '\n', self.dest)
        return True


class ControlInterface(object):
    PORT = 13356

    TIMEOUT = 1.0

    def __init__(self, opts):
        self.logger = logging.getLogger('control')

        self.opts = opts

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setblocking(0)
        self.logger.info('Binding to port %d' % self.PORT)
        self.sock.bind(('', self.PORT))

        self.src_addr = None
        self.recent_packets = 0
        self.last_seq = None
        
        if self.opts.no_mech:
            self.mech_driver = None
        else:
            self.mech_driver = gait_driver.MechDriver(opts)
            Task(self.mech_driver.run())

        self.video_addr = None
        self.video_proc = None
        gobject.timeout_add(int(self.TIMEOUT * 1000),
                            wrap_event(self._on_timeout))
        gobject.io_add_watch(self.sock.fileno(), 
                             gobject.IO_IN | gobject.IO_ERR | gobject.IO_HUP,
                             wrap_event(self._on_readable))

    def _on_timeout(self):
        if self.recent_packets:
            self.logger.debug('Remote peer %r sent %d packet(s)' % (
                    self.src_addr, self.recent_packets))
            self.recent_packets = 0
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
        self.logger.info('Remote peer address is now %r' % (addr, ))
        self.src_addr = addr
        self._set_video_dest(None)

    def _handle_packet(self, pkt_bin):
        self.recent_packets += 1
        pkt = json.loads(pkt_bin)

        vport = pkt.get('video_port', 0)
        if not vport:
            self._set_video_dest(None)
        else:
            self._set_video_dest((self.src_addr[0], vport))

        if (pkt['seq'] - 1) != self.last_seq:
            self.logger.info('Seq number jump: %r->%r' % (self.last_seq, pkt['seq']))
        self.last_seq = pkt['seq']

        if 'servo_x' in pkt and 'servo_y' in pkt and self.mech_driver:
            Task(self.mech_driver.servo.set_pose(
                    {12: pkt['servo_x'],
                     13: pkt['servo_y']}))

        if 'gait' in pkt and self.mech_driver:
            gait = pkt['gait']
            if gait is None:
                self.mech_driver.set_idle()
            else:
                command = legtool.gait.ripple.Command()
                for key, value in gait.iteritems():
                    setattr(command, key, value)

                self.mech_driver.set_command(command)

        #self.logger.debug('Remote packet: %r' % (pkt, ))

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
        self.logger.info('Started video process, PID %r' % pid)
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
    asyncio.set_event_loop_policy(gbulb.GLibEventLoopPolicy())

    logging.basicConfig(
        level=logging.WARN,
        format="%(asctime)s.%(msecs).3d [%(levelname).1s] %(name)s: %(message)s",
        datefmt="%T")

    if opts.check:
        logging.info('Check passed')
        return
    # cleanup hack
    os.system("killall -v gst-launch-1.0")

    ann = UdpAnnouncer()
    cif = ControlInterface(opts)
    logging.info('Running')

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
