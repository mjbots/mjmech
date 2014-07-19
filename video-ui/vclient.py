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

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
from gi.repository import GObject, GLib
from gi.repository import GdkX11, GstVideo, Gtk, Gdk

# based on example at:
# http://bazaar.launchpad.net/~jderose/+junk/gst-examples/view/head:/video-player-1.0
# docs:
# http://pygstdocs.berlios.de/pygst-reference/gst-class-reference.html

def wrap_event(callback):
    """Wrap event callback so the app exit if it crashes"""
    def wrapped(*args, **kwargs):
        try:
            return callback(*args, **kwargs)
        except BaseException as e:
            logging.error("Callback %r crashed:", callback)
            logging.error(" %s %s" % (e.__class__.__name__, e))
            Gtk.main_quit()
            raise
    return wrapped

NULL_COMMAND = {
    'translate_x_mm_s': 0,
    'translate_y_mm_s': 0,
    'rotate_deg_s': 0,
    }

class UdpAnnounceReceiver(object):
    PORT = 13355

    def __init__(self, opts):
        self.opts = opts
        self.logger = logging.getLogger('announce-receiver')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setblocking(0)
        self.logger.info('Binding to port %d' % self.PORT)
        self.sock.bind(('', self.PORT))

        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        GLib.io_add_watch(self.sock.fileno(), 
                          GLib.PRIORITY_DEFAULT,
                          GLib.IO_IN | GLib.IO_ERR | GLib.IO_HUP,
                          self._on_readable)
        self.logger.info('Waiting for address broadcast')
        # active server info
        self.server = None
        self.recent_sources = list()
        self.control = None
        
    @wrap_event
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
            self.control = ControlInterface(self.opts, self.server['addr'], 
                                            self.server['cport'])

class ControlInterface(object):
    SEND_INTERVAL = 0.25
    VIDEO_PORT = 13357

    def __init__(self, opts, host, port=13356):
        self.opts = opts
        self.logger = logging.getLogger('control')
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

        self.sock.setblocking(0)
        self.logger.info('Connecting to %s:%d' % self.addr)
        self.sock.connect(self.addr)

        self.seq = 0
        self.control_dict = dict(
            video_port=self.VIDEO_PORT,
            servo_x = 0.5, servo_y = 0.5,
            )
       
        if opts.external_video:
            self.video = None
        else:
            self.video = VideoWindow(host, self.VIDEO_PORT)
            self.video.on_video_click_1 = self._handle_video_click_1
            self.video.on_key_press = self._handle_key_press
            self.video.on_key_release = self._handle_key_release

        GLib.timeout_add(int(self.SEND_INTERVAL * 1000),
                         self._on_send_timer)
        GLib.io_add_watch(self.sock.fileno(), 
                          GLib.PRIORITY_DEFAULT,
                          GLib.IO_IN | GLib.IO_ERR | GLib.IO_HUP,
                          self._on_readable)

    @wrap_event
    def _on_send_timer(self):
        self._send_control()
        return True

    @wrap_event
    def _on_readable(self, source, cond):
        while True:
            try:
                pkt, addr = self.sock.recvfrom(65535)
            except socket.error as e:
                if e.errno != errno.EAGAIN:
                    raise
                break
            assert addr == self.addr, \
                'Got packet from %r, expected from %r' % (addr, self.addr)
            self._handle_packet(pkt)
        return True

    def _send_control(self):
        self.seq += 1
        pkt = self.control_dict.copy()
        pkt.update(seq=self.seq)
        self.sock.send(json.dumps(pkt))
        if self.seq == 3 and self.video:
            self.video.start()

    def _handle_packet(self, pkt):
        self.logger.info('Remote packet: %r' % (pkt, ))

    def _handle_video_click_1(self, pos):
        """User clicked button 1 on video window.
        pos is (x, y) tuple in 0..1 range"""
        self.control_dict['servo_x'] = pos[0]
        self.control_dict['servo_y'] = pos[1]
        self._send_control()

    def _handle_key_press(self, evt):
        dx = 0
        dy = 0
        dr = 0
        name = Gdk.keyval_name(evt.keyval)
        if name == 'w':
            dy = 1
        elif name == 's':
            dy = -1
        elif name == 'a':
            dx = -1
        elif name == 'd':
            dx = 1
        elif name == 'q':
            dr = -1
        elif name == 'e':
            dr = 1

        gait = NULL_COMMAND.copy()
        gait['translate_x_mm_s'] = dx * 50
        gait['translate_y_mm_s'] = dy * 100
        gait['rotate_deg_s'] = dr * 30
        self.control_dict['gait'] = gait

    def _handle_key_release(self, evt):
        self.control_dict['gait'] = None

class VideoWindow(object):
    def __init__(self, host, port, rotate=False):
        self.host = host
        self.port = port
        self.logger = logging.getLogger('video')

        self.window = Gtk.Window()
        self.window.set_title("VClient %s:%d" % (self.host, self.port))
        self.window.set_default_size(500, 400)
        self.window.connect("destroy", self.quit)
        vbox = Gtk.VBox()
        self.window.add(vbox)

        self.drawingarea = Gtk.DrawingArea()
        self.drawingarea.add_events(Gdk.EventMask.BUTTON_PRESS_MASK |
                                    Gdk.EventMask.BUTTON_RELEASE_MASK |
                                    Gdk.EventMask.POINTER_MOTION_MASK
                                    )
        self.drawingarea.connect("button-press-event", self._on_da_click)        
        self.drawingarea.connect("motion-notify-event", self._on_da_move)
        self.window.connect("key-press-event", self._on_da_key)
        self.window.connect("key-release-event", self._on_da_release)
        vbox.add(self.drawingarea)
        self.window.show_all()
        # You need to get the XID after window.show_all().  You shouldn't get it
        # in the on_sync_message() handler because threading issues will cause
        # segfaults there.
        self.xid = self.drawingarea.get_property('window').get_xid()

        self.pipeline = Gst.Pipeline()
        self.rtpbin = self.make_element("rtpbin", do_retransmission=True)
        
        #caps = Gst.Caps("application/x-rtp",
        #                media="video", clock_rate=90000, encoding_name="H264")
        caps = Gst.Caps.from_string("application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264")
        rtp_src = self.make_element("udpsrc", caps=caps, port=self.port)
        self.link_pads(rtp_src, None, self.rtpbin, "recv_rtp_sink_0")

        rtcp_src = self.make_element("udpsrc", port=self.port + 1)
        self.link_pads(rtcp_src, None, self.rtpbin, "recv_rtcp_sink_0")
        
        rtcp_sink = self.make_element("udpsink", host=self.host, port=self.port + 2,
                                      sync=False, async=False)
        self.link_pads(self.rtpbin, "send_rtcp_src_0", rtcp_sink, None)

        self.textoverlay = self.make_element(
            "textoverlay",
            shaded_background=True, font_desc="16", auto_resize=False,
            valignment="bottom", halignment="left", line_alignment="left",
            text="Video info\nNo data yet")

        play_elements = [
            self.make_element("rtph264depay"),
            self.make_element("h264parse"),
            self.make_element("avdec_h264"),
            self.make_element("videoconvert"),
            self.make_element("timeoverlay", shaded_background=True,
                              font_desc="8",
                              valignment="bottom", halignment="right"),
            self.textoverlay,
            ]
        if rotate:
            play_elements.append(self.make_element(
                    "videoflip", method="clockwise"))
        play_elements.append(self.make_element("xvimagesink"))
        for i in range(1, len(play_elements)):
            self.link_pads(play_elements[i-1], None, play_elements[i], None)
        self.play_elements = play_elements
        self.rtpbin.connect("pad-added", self._on_new_rtpbin_pad)

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_error)
        bus.connect("message::eos", self._on_end_of_stream)
        bus.enable_sync_message_emission()
        bus.connect('sync-message::element', self._on_sync_message)       

        # Callback functions
        self.on_video_click_1 = None
        self.on_key_press = None
        self.on_key_release = None

    def make_element(self, etype, **kwargs):
        elt = Gst.ElementFactory.make(etype)        
        assert elt
        self.pipeline.add(elt)
        for n, v in sorted(kwargs.items()):
            elt.set_property(n.replace('_', '-'), v)
        return elt

    def link_pads(self, elt1, pad1, elt2, pad2):
        if pad1 is None and pad2 is None:
            res = elt1.link(elt2)
        else:
            res = elt1.link_pads(pad1, elt2, pad2)
        assert res, (elt1, pad1, elt2, pad2)    

    def start(self):
        self.logger.info('Starting video')
        self.pipeline.set_state(Gst.State.PLAYING)

    def quit(self, sender):
        self.logger.info('quitting -- window closed')
        self.pipeline.set_state(Gst.State.NULL)
        Gtk.main_quit()

    @wrap_event
    def _on_new_rtpbin_pad(self, source, pad):
        name = pad.get_name()
        self.logger.info('Got new rtpbin pad: %r' % (name, ))
        if name.startswith('recv_rtp_src'):
            self.link_pads(self.rtpbin, name, self.play_elements[0], None)

    @wrap_event
    def _on_sync_message(self, bus, msg):
        if msg.get_structure().get_name() == 'prepare-window-handle':
            self.logger.info('Embedding video window')
            msg.src.set_window_handle(self.xid)

    @wrap_event
    def _on_error(self, bus, msg):
        err, debug = msg.parse_error()
        self.logger.error("GstError: %s" % err)
        if debug:
            for line in debug.split('\n'):
                self.logger.info('| %s' % line)

    @wrap_event
    def _on_end_of_stream(self, bus, msg):
        self.logger.info("End of stream: %r" % (msg, ))


    def _evt_get_video_coord(self, evt):
        # Get size of window
        alloc = self.drawingarea.get_allocation()
        wnd_size = (alloc.width, alloc.height)
        #wnd_size = (self.play_elements[-1].get_property('window-width'),
        #            self.play_elements[-1].get_property('window-height'))

        # Get original size of video stream
        caps = self.play_elements[-1].sinkpad.get_current_caps()
        if caps is None:
            return None
        struct = caps.get_structure(0)  # Assume these are simple caps with a single struct.
        video_size = (struct.get_int('width')[1], struct.get_int('height')[1])

        # Calculate image position in (-1..1) range (taking in the account that video is
        # scaled, but aspect ratio is preserved)
        scale = min(wnd_size[0] * 1.0 / video_size[0],
                    wnd_size[1] * 1.0 / video_size[1])
        rel_pos = (0.5 + (evt.x - wnd_size[0]/2.0) / scale / video_size[0],
                   0.5 + (evt.y - wnd_size[1]/2.0) / scale / video_size[1])
        if (0 <= rel_pos[0] <= 1 and 0 <= rel_pos[1] <= 1):
            return rel_pos
        else:
            return None

    @wrap_event
    def _on_da_move(self, src, evt):
        assert src == self.drawingarea, src
        #self.logger.info('DrawingArea move: %r' % ((evt.x, evt.y, evt.state, ), ))

        if evt.state & Gdk.ModifierType.BUTTON1_MASK:
            # Button is being held.
            rel_pos = self._evt_get_video_coord(evt)
            if rel_pos and self.on_video_click_1:
                self.on_video_click_1(rel_pos)

    @wrap_event
    def _on_da_click(self, src, evt):
        assert src == self.drawingarea, src
        rel_pos = self._evt_get_video_coord(evt)
        if rel_pos is None:
            self.logger.info('Video click outside of image at wpt=(%d,%d) button=%d state=%d' % 
                             (evt.x, evt.y, evt.button, evt.state))
            return True

        self.logger.info('Video click at wpt=(%d,%d) rel=(%.3f,%.3f) button=%d state=%d' % 
                         (evt.x, evt.y, rel_pos[0], rel_pos[1], evt.button, evt.state))
        if evt.button == 1 and self.on_video_click_1:
            self.on_video_click_1(rel_pos)
        return True

    @wrap_event
    def _on_da_key(self, src, evt):
        assert src == self.window, src
        if self.on_key_press:
            self.on_key_press(evt)
        return True

    @wrap_event
    def _on_da_release(self, src, evt):
        assert src == self.window, src
        if self.on_key_release:
            self.on_key_release(evt)
        return True

def main(opts):
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s.%(msecs).3d [%(levelname).1s] %(name)s: %(message)s",
        datefmt="%T")

    if opts.check:
        logging.info('Check passed')
        return

    #GObject.threads_init()
    Gst.init(None)

    if opts.addr is None:
        ann = UdpAnnounceReceiver(opts)
    else:
        cif = ControlInterface(opts, opts.addr)

    logging.info('Running')
    Gtk.main()

if __name__ == '__main__':
    parser = optparse.OptionParser()
    parser.add_option('--check', action='store_true',
                      help='Exit immediately')
    parser.add_option('--addr', metavar='IP',
                      help='Client address')
    parser.add_option('--external-video', action='store_true',
                      help='')

    opts, args = parser.parse_args()
    if len(args) and opts.addr is None:
        opts.addr = args.pop(0)
    if args:
        parser.error('Invalid number of args')
    main(opts)
