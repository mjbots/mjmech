#!/usr/bin/python
import sys
import os
import gobject
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

class SerialControl(object):
    """Serial control object.
    Assumes Firmata control interface with servoes on pins 3 and 5
    """

    SERPORT = '/dev/ttyACM99'
    BAUDRATE = 57600

    def __init__(self):
        self.pos = (0.5, 0.5)
        self.send_count = 0

        # We are not using dev's read/write methods, as they
        # are not safe to use inside the loop.
        # instead, we just use underlying fd directly.
        self.serial_dev = serial.Serial(self.SERPORT, baudrate=self.BAUDRATE)
        self.serial_fd = self.serial_dev.fileno()

        # Assert that the fd is nonblocking
        mode = fcntl.fcntl(self.serial_fd, fcntl.F_GETFL)
        assert mode & os.O_NONBLOCK, mode

    def on_timer(self):
        self._send_packet()

    def set_pos(self, x, y):
        new_pos = (min(1, max(0, x)),
                   min(1, max(0, y)))
        if new_pos == self.pos:
            return
        self.pos = new_pos
        self._send_packet()

    def _send_packet(self):
        data = []
        if (self.send_count % 20) == 0:
            # Pins 3 and 5 to servo mode
            data += [0xF4, 3, 4, 0xF4, 5, 4]
        # map 0..1 positions to degrees for firmata
        dvals = (int(60 + 60 * self.pos[0]),
                 int(60 + 60 * self.pos[1]))
        for pin, d_val in zip([3, 5], dvals):
            data += [0xE0 | pin, d_val & 0x7F, (d_val >> 7) & 0x7F]
        data_bin = ''.join(chr(x) for x in data)
        written = os.write(self.serial_fd, data_bin)
        assert written == len(data_bin), (written, len(data_bin))

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
        
        if self.opts.no_serial:
            self.serial_control = None
        else:
            self.serial_control = SerialControl()

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
        if self.serial_control:
            self.serial_control.on_timer()
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

        if 'servo_x' in pkt and 'servo_y' in pkt and self.serial_control:
            self.serial_control.set_pos(pkt['servo_x'], pkt['servo_y'])

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
    logging.basicConfig(
        level=logging.DEBUG,
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
    g_main_loop.run()

if __name__ == '__main__':
    parser = optparse.OptionParser()
    parser.add_option('--check', action='store_true',
                      help='Exit immediately')
    parser.add_option('--no-serial', action='store_true',
                      help='Do not open serial control port')
    opts, args = parser.parse_args()
    if args:
        parser.error('No args accepted')
    main(opts)
