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

class UdpAnnounceReceiver(object):
    PORT = 13355

    def __init__(self):
        self.logger = logging.getLogger('announce-receiver')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setblocking(0)
        self.logger.info('Binding to port %d' % self.PORT)
        self.sock.bind(('', self.PORT))

        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        gobject.io_add_watch(self.sock.fileno(), 
                             gobject.IO_IN | gobject.IO_ERR | gobject.IO_HUP,
                             wrap_event(self._on_readable))
        self.logger.info('Waiting for address broadcast')
        # active server info
        self.server = None
        self.recent_sources = list()
        self.control = None
        
    def _on_readable(self, source, cond):
        while True:
            try:
                pkt, addr = self.sock.recvfrom(65535)
            except socket.error as e:
                if e.errno != errno.EAGAIN:
                    raise
                break
            data = json.loads(pkt)
            if (self.server is None or 
                self.server['host'] != data['host'] or
                self.server['start_time'] != data['start_time'] or
                addr not in self.recent_sources):
                # The server got restarted, or source address has changed.
                server = dict(host=data['host'],
                              start_time=data['start_time'],
                              cport=data['cport'], # control port
                              addr=addr[0], 
                              aport=addr[1] # announce port
                              )
                self._on_new_server(server)
                self.recent_sources.append(addr)
            while len(self.recent_sources) > 10:
                self.recent_sources.pop(0)
        return True

    def _on_new_server(self, info):
        self.logger.info('Found a server at %s:%d (a-port %s, host %r, started %s)' % (
                info['addr'], info['cport'], info['aport'], info['host'], 
                time.strftime('%F_%T', time.localtime(info['start_time']))))
        assert self.server is None or self.server['addr'] == info['addr'], \
            'IP change not supported'
        self.server = info
        if self.control is None:
            self.control = ControlInterface(self.server['addr'], self.server['cport'])

class ControlInterface(object):
    SEND_INTERVAL = 0.25
    VIDEO_PORT = 13357

    def __init__(self, host, port):
        self.logger = logging.getLogger('control')
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setblocking(0)
        self.logger.info('Connecting to %s:%d' % self.addr)
        self.sock.connect(self.addr)
        self.seq = 0
       
        gobject.timeout_add(int(self.SEND_INTERVAL * 1000),
                            wrap_event(self._on_send_timer))
        gobject.io_add_watch(self.sock.fileno(), 
                             gobject.IO_IN | gobject.IO_ERR | gobject.IO_HUP,
                             wrap_event(self._on_readable))

    def _on_send_timer(self):
        self.seq += 1
        pkt = dict(video_port=self.VIDEO_PORT,
                   seq=self.seq)
        self.sock.send(json.dumps(pkt))
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

    def _handle_packet(self, pkt):
        self.recent_packets += 1
        self.logger.info('Remote packet: %r' % (pkt, ))

def main(opts):
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s.%(msecs).3d [%(levelname).1s] %(name)s: %(message)s",
        datefmt="%T")

    if opts.check:
        logging.info('Check passed')
        return

    if opts.addr is None:
        ann = UdpAnnounceReceiver()
    else:
        pass

    #ann = UdpAnnouncer()
    #cif = ControlInterface()
    logging.info('Running')
    g_main_loop.run()

if __name__ == '__main__':
    parser = optparse.OptionParser()
    parser.add_option('--check', action='store_true',
                      help='Exit immediately')
    parser.add_option('--addr', metavar='IP',
                      help='Client address')
    opts, args = parser.parse_args()
    if len(args) and opts.addr is None:
        opts.addr = args.pop(0)
    if args:
        parser.error('Invalid number of args')
    main(opts)
