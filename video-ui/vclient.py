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
import cStringIO as StringIO
import functools

import trollius as asyncio
from trollius import Task, From

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
from gi.repository import GObject, GLib
from gi.repository import GdkX11, GstVideo, Gtk, Gdk

sys.path.append(os.path.join(os.path.dirname(__file__), '../'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../legtool/'))

import gbulb
import joystick
import legtool.async.trollius_trace
import linux_input

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

def add_pair(a, b):
    return (a[0] + b[0], a[1] + b[1])

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
    UI_STATE_SAVEFILE = "last.jsonlist"

    def __init__(self, opts, host, port=13356):
        self.opts = opts
        self.logger = logging.getLogger('control')
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

        self.sock.setblocking(0)
        self.logger.info('Connecting to %s:%d' % self.addr)
        self.sock.connect(self.addr)

        # Wallclock and pipeline time
        self.last_video_wall_time = None
        self.last_video_pp_time = None

        self.seq = 0
        self.control_dict = dict(
            video_port=self.VIDEO_PORT,
            # Not set: turret - pair of (x, y) degrees
            # Not set: gait - a dict based off NULL_COMMAND

            # 0 = laser off; 1 = laser on
            laser_on = 0,

            # fire a shot every time this changes; should only ever increase
            fire_cmd_count = 0,
            mixer_on = 0,
            )

        # state log file and last svg
        self.state_log_file = None
        self.last_svg_name = None
        self.state_savefile_name = None
        if opts.log_prefix:
            # Last svg needs no timestamp
            self.last_svg_name = os.path.join(
                os.path.dirname(opts.log_prefix + 'test'), 'last.svg')
            # Open file line-buffered
            self.state_log_file = open(
                opts.log_prefix + '.jsonlist', 'w', 1)
            # Prepare savefile name
            self.state_savefile_name = os.path.join(
                os.path.dirname(opts.log_prefix + 'test'),
                self.UI_STATE_SAVEFILE)

        # Control UI state. Should be a simple dict, as we will be serializing
        # it for later log replay.
        self.state = dict(self._INITIAL_STATE)
        self._logged_state = None

        restore_from = opts.restore_state

        # default -- only restore if file exists
        if (restore_from is None
            and self.state_savefile_name is not None
            and os.path.exists(self.state_savefile_name)):
            restore_from = self.state_savefile_name

        if restore_from:    # not empty string or None
            # Restore state
            logging.info('Loading saved state from %r', restore_from)
            with open(restore_from, 'r') as fh:
                restored = json.load(fh)
            self.state.update(restored)

        self.video = None

        # Log state before video, to check for code errors in rendering function
        self._state_updated()


        if not opts.external_video:
            video_log = None
            if opts.log_prefix:
                video_log = opts.log_prefix + '.mkv'
            self.video = VideoWindow(host, self.VIDEO_PORT, video_log=video_log)
            self.video.on_video_click_1 = self._handle_video_click_1
            self.video.on_key_press = self._handle_key_press
            self.video.on_key_release = self._handle_key_release
            self.video.on_got_video = functools.partial(
                self._state_updated, force=True)

        def metric(device):
            abs_axis = device.get_features(linux_input.EV.ABS)
            for x in [linux_input.ABS.X,
                      linux_input.ABS.Y,
                      linux_input.ABS.RX]:
                if not x in abs_axis:
                    return -1
            return 1

        enumerator = joystick.JoystickEnumerator(metric)
        joysticks = enumerator.joysticks()
        if len(joysticks):
            self.joystick = joysticks[0]
            self.joystick_task = Task(self.read_joystick())
        else:
            self.joystick = None
            self.joystick_task = None
            self.logger.info("No joysticks found!")


        GLib.timeout_add(int(self.SEND_INTERVAL * 1000),
                         self._on_send_timer)
        GLib.io_add_watch(self.sock.fileno(),
                          GLib.PRIORITY_DEFAULT,
                          GLib.IO_IN | GLib.IO_ERR | GLib.IO_HUP,
                          self._on_readable)

    @asyncio.coroutine
    def read_joystick(self):
        try:
            while True:
                ev = yield From(self.joystick.read())

                if ev.ev_type != linux_input.EV.ABS:
                    continue

                dx = self.joystick.absinfo(linux_input.ABS.X).scaled()
                dy = self.joystick.absinfo(linux_input.ABS.Y).scaled()
                dr = self.joystick.absinfo(linux_input.ABS.RX).scaled()

                if abs(dx) < 0.2 and abs(dy) < 0.2 and abs(dr) < 0.2:
                    self.control_dict['gait'] = None
                else:
                    gait = NULL_COMMAND.copy()
                    gait['translate_x_mm_s'] = dx * 50
                    gait['translate_y_mm_s'] = -dy * 200
                    gait['rotate_deg_s'] = dr * 50
                    self.control_dict['gait'] = gait
        except Exception as e:
            print >>sys.stderr, "ERROR!:", str(e)
            raise

    @wrap_event
    def _on_send_timer(self):
        if self.joystick_task:
            if self.joystick_task.done():
                self.joystick_task.result()
                self.joystick_task = None

        self._send_control()

        if self.video:
            tm_pp = self.video.get_time()
            tm_wc = time.time()
            if self.last_video_pp_time is not None:
                dt_pp = tm_pp - self.last_video_pp_time
                dt_wc = tm_wc - self.last_video_wall_time
                lost = (dt_wc - dt_pp)
                if abs(lost) > 0.01:
                    self.logger.info('Video time lost: %.3f msec (%.1f%%)' % (
                        lost * 1000.0, lost * 100.0 / (dt_wc or 1)))

            self.last_video_wall_time = tm_wc
            self.last_video_pp_time = tm_pp
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

    _CAMERA_VIEW_ANGLE_DIAG = 78.0   # from camera datasheet
    _CAMERA_VIEW_ANGLE_HOR = _CAMERA_VIEW_ANGLE_DIAG / (16*16 + 9*9)**(0.5) * 16
    _CAMERA_VIEW_ANGLE_VERT = _CAMERA_VIEW_ANGLE_DIAG / (16*16 + 9*9)**(0.5) * 9

    def _handle_video_click_1(self, pos, moved=False):
        """User clicked button 1 on video window.
        pos is (x, y) tuple in 0..1 range"""
        assert len(pos) == 2
        if moved:
            # Only care about initial click.
            return
        old_turret = self.control_dict.get('turret')
        if old_turret is None:
            self.logger.warn('Cannot move turret -- center it first')
            return
        ang_x, ang_y = old_turret

        ang_x -= (pos[0] - 0.5 - self.state['reticle_offset'][0]) \
                 * self._CAMERA_VIEW_ANGLE_HOR
        ang_y += (pos[1] - 0.5 - self.state['reticle_offset'][1]) \
                 * self._CAMERA_VIEW_ANGLE_VERT
        # TODO mafanasyev: add reticle_rotate support

        self.logger.info('Setting turret to (%+.1f, %+.1f) deg in response '
                         'to click at (%.4f, %.4f)',
                         ang_x, ang_y, pos[0], pos[1])
        self.control_dict['turret'] = (ang_x, ang_y)
        self._send_control()

    _GAIT_KEYS = {  #  key->(dx,dy,dr)
        'w': (0, -1, 0),
        's': (0, 1, 0),
        'a': (-1, 0, 0),
        'd': (1, 0, 0),
        'q': (0, 0, -1),
        'e': (0, 0, 1)
    }

    _ARROW_KEYS = { # key->(dx, dy)
        'Left': (-1, 0),
        'Right': (1, 0),
        'Up': (0, -1),
        'Down': (0, 1)
    }

    def _handle_key_press(self, evt):
        # Parse out the keys
        base_name = Gdk.keyval_name(evt.keyval)
        modifiers = ''
        MT = Gdk.ModifierType
        if evt.state & MT.CONTROL_MASK: modifiers += 'C-'
        if evt.state & MT.SHIFT_MASK: modifiers += 'S-'
        if evt.state & MT.MOD1_MASK: modifiers += 'M-'
        name = modifiers + base_name

        if name in self._GAIT_KEYS:
            dx, dy, dr = self._GAIT_KEYS[name]
            gait = NULL_COMMAND.copy()
            gait['translate_x_mm_s'] = dx * 50
            gait['translate_y_mm_s'] = dy * 100
            gait['rotate_deg_s'] = dr * 30
            self.control_dict['gait'] = gait
        elif name == 'h':
            self._print_help()
        elif name == 'l':
            self.control_dict['laser_on'] ^= 1
            self.logger.info('Laser set to %d',
                             self.control_dict['laser_on'])
        elif name == 'm':
            self.control_dict['mixer_on'] ^= 1
            self.logger.info('Mixer set to %d',
                             self.control_dict['mixer_on'])
        elif name in ['Return']:
            self.control_dict['fire_cmd_count'] += 1
            self.logger.info('Sent fire command')
        elif name == 'c':
            self.control_dict['turret'] = (0.0, 0.0)
            self.logger.info('Centered turret')
        elif base_name in self._ARROW_KEYS and 'C-' in modifiers:
            # Ctrl + arrows to move reticle center
            dx, dy = self._ARROW_KEYS[base_name]
            # Shift makes it slower
            if 'S-' in modifiers:
                step = 0.002
            else:
                step = 0.010
            self.state['reticle_offset'] = add_pair(
                self.state['reticle_offset'], (step * dx, step * dy))
        elif name == 'r':
            self.state['reticle_on'] ^= True
            self.logger.info('Set reticle_on=%r', self.state['reticle_on'])
        else:
            self.logger.info('Unknown key %r' % name)

        self._state_updated()
        self._send_control()

    def _handle_key_release(self, evt):
        if self.control_dict.get('gait') is not None:
            self.control_dict['gait'] = None
            self._send_control()

    def _print_help(self):
        print 'Keys:'
        print '  w/s, a/d - move'
        print '  q/e      - rotate'
        print '  l        - laser on/off'
        print '  m        - mixer on/off'
        print '  c        - enable && center turret'
        print '  click    - point turret to this location (must center first)'
        print '  ENTER    - fire'
        print '  C+arrows - set reticle center (use shift for more precision)'
        print '  r        - toggle reticle'

    def _state_updated(self, force=False):
        sertext = json.dumps(self.state, sort_keys=True)
        if (sertext == self._logged_state) and not force:
            return
        self._logged_text = sertext

        if self.state_log_file:
            # TODO mafanasyev: add video timestamp
            print >>self.state_log_file, sertext
            # No need to flush

        if self.state_savefile_name is not None:
            with open(self.state_savefile_name + '~', 'w') as fh:
                print >>fh, sertext
            # Atomically install new version
            os.rename(self.state_savefile_name + '~',
                      self.state_savefile_name)


        stream = StringIO.StringIO()
        self._render_svg(stream, self.state)
        if self.video:
            self.video.set_svg_overlay(stream.getvalue())

        if self.last_svg_name:
            with open(self.last_svg_name + '~', 'w') as f:
                f.write(stream.getvalue())
            os.rename(self.last_svg_name + '~', self.last_svg_name)


    _INITIAL_STATE = dict(
        # SVG is stretched to video after rendering, so small width/height
        # will make all lines thicker. Force to video resolution: 1920x1080
        image_size=(1928, 1080),
        reticle_on=True,
        reticle_offset=(0, 0),
        reticle_rotate=0,
    )

    @staticmethod
    def _render_svg(out, state):
        """Render SVG for a given state. Should not access anything other
        than parameters, as this will be called during replay.
        """
        print >>out, '<svg width="{image_size[0]}" height="{image_size[1]}">'\
            .format(**state)

        if state['reticle_on']:
            reticle_center_rel = add_pair((0.5, 0.5), state['reticle_offset'])
            reticle_center = (
                state['image_size'][0] * reticle_center_rel[0],
                state['image_size'][1] * reticle_center_rel[1])

            print >>out, '''
<g transform='rotate({0[reticle_rotate]}) translate({1[0]} {1[1]})'
   stroke="rgb(255,128,0)">
  <line x1="500"  x2="100"  y1="0" y2="0" stroke-width="4" />
  <line x1="-500" x2="-100" y1="0" y2="0" stroke-width="4" />
  <line x1="-100" x2="100"  y1="0" y2="0" />
  <line y1="500"  y2="100"  x1="0" x2="0" stroke-width="4" />
  <line y1="-500" y2="-100" x1="0" x2="0" stroke-width="4" />
  <line y1="-100" y2="100"  x1="0" x2="0" />

  <line x1="-80" x2="80"  y1="-20" y2="-20" />
  <line x1="-80" x2="80"  y1="20" y2="20" />
  <line x1="-60" x2="60"  y1="40" y2="40" />
  <line x1="-40" x2="40"  y1="60" y2="60" />
  <line x1="-20" x2="20"  y1="80" y2="80" />
</g>
'''.format(state, reticle_center)

        print >>out, '<text x="0" y="15" fill="red">Logs will go here</text>'
        print >>out, '</svg>'


class VideoWindow(object):
    def __init__(self, host, port, rotate=False, video_log=None):
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

        caps = Gst.Caps.from_string(
            "application/x-rtp,media=(string)video,clock-rate=(int)90000,"
            "encoding-name=(string)H264")
        rtp_src = self.make_element("udpsrc", caps=caps, port=self.port)
        self.link_pads(rtp_src, None, self.rtpbin, "recv_rtp_sink_0")

        rtcp_src = self.make_element("udpsrc", port=self.port + 1)
        self.link_pads(rtcp_src, None, self.rtpbin, "recv_rtcp_sink_0")

        rtcp_sink = self.make_element("udpsink", host=self.host, port=self.port + 2,
                                      sync=False, async=False)
        self.link_pads(self.rtpbin, "send_rtcp_src_0", rtcp_sink, None)

        #self.info_overlay = self.make_element(
        #    "textoverlay",
        #    shaded_background=True, font_desc="16", auto_resize=False,
        #    valignment="bottom", halignment="left", line_alignment="left",
        #    text="Video info\nNo data yet")
        self.info_overlay = self.make_element(
            "rsvgoverlay",
            fit_to_frame=True)

        decode_elements = [
            self.make_element("rtph264depay"),
            self.make_element("h264parse"),
            self.make_element("tee"),
        ]
        play_elements = decode_elements + [
            self.make_element('queue'),
            self.make_element("avdec_h264"),
            self.make_element("videoconvert"),
            self.make_element("timeoverlay", shaded_background=True,
                              font_desc="8",
                              valignment="bottom", halignment="right"),
            self.info_overlay,
            self.make_element("videoconvert"),
            ]
        if rotate:
            play_elements.append(self.make_element(
                    "videoflip", method="clockwise"))
        play_elements.append(self.make_element("xvimagesink"))
        self.link_list_of_pads(play_elements)
        self.play_elements = play_elements
        self.rtpbin.connect("pad-added", self._on_new_rtpbin_pad)


        if video_log is not None:
            self.logger.info('Recording video to %r' % video_log)
            save_elements = [
                decode_elements[-1],
                self.make_element('queue'),
                self.make_element('matroskamux'),
                self.make_element('filesink', location=video_log)
            ]
            self.link_list_of_pads(save_elements)

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
        self.on_got_video = None

    def make_element(self, etype, **kwargs):
        elt = Gst.ElementFactory.make(etype)
        assert elt, 'Failed to make element of type %r' % etype
        self.pipeline.add(elt)
        for n, v in sorted(kwargs.items()):
            elt.set_property(n.replace('_', '-'), v)
        return elt

    def link_pads(self, elt1, pad1, elt2, pad2):
        if pad1 is None and pad2 is None:
            res = elt1.link(elt2)
        else:
            res = elt1.link_pads(pad1, elt2, pad2)
        assert res, 'Failed to link (%r,%r) to (%r,%r)' % (
            elt1, pad1, elt2, pad2)

    def link_list_of_pads(self, elements):
        for i in range(1, len(elements)):
            self.link_pads(elements[i-1], None, elements[i], None)

    def start(self):
        self.logger.info('Starting video')
        self.pipeline.set_state(Gst.State.PLAYING)

    def quit(self, sender):
        self.logger.info('quitting -- window closed')
        self.pipeline.set_state(Gst.State.NULL)
        Gtk.main_quit()

    def get_time(self):
        """Return timestamp in video. This freezes when video does not work.
        """
        clock = self.pipeline.get_clock()
        tm = clock.get_internal_time()
        return tm / 1.e9

    @wrap_event
    def _on_new_rtpbin_pad(self, source, pad):
        name = pad.get_name()
        self.logger.info('Got new rtpbin pad: %r' % (name, ))
        if name.startswith('recv_rtp_src'):
            self.link_pads(self.rtpbin, name, self.play_elements[0], None)

    @wrap_event
    def _on_sync_message(self, bus, msg):
        struct_name = msg.get_structure().get_name()
        if struct_name == 'prepare-window-handle':
            self.logger.info('Embedding video window')
            msg.src.set_window_handle(self.xid)
        else:
            self.logger.debug('sync message: %r' % struct_name)
            if self.on_got_video:
                self.on_got_video()

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
                self.on_video_click_1(rel_pos, moved=True)

    @wrap_event
    def _on_da_click(self, src, evt):
        assert src == self.drawingarea, src
        rel_pos = self._evt_get_video_coord(evt)
        if rel_pos is None:
            self.logger.info(
                'Video click outside of image at wpt=(%d,%d) button=%d '
                'state=%d', evt.x, evt.y, evt.button, evt.state)
            return True

        self.logger.debug(
            'Video click at wpt=(%d,%d) rel=(%.3f,%.3f) button=%d state=%d',
            evt.x, evt.y, rel_pos[0], rel_pos[1], evt.button, evt.state)
        if evt.button == 1 and self.on_video_click_1:
            self.on_video_click_1(rel_pos, moved=False)
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

    def set_svg_overlay(self, data):
        self.info_overlay.set_property("data", data)


def main(opts):
    asyncio.set_event_loop_policy(gbulb.GLibEventLoopPolicy())

    logging.basicConfig(
        level=logging.DEBUG,
        format=("%(asctime)s.%(msecs).3d "
                "[%(levelname).1s] %(name)s: %(message)s"),
        datefmt="%T")

    root = logging.getLogger()
    # Do not apply limits on loggers; instead apply them on handlers.
    root.setLevel(logging.DEBUG)

    if opts.check:
        logging.info('Check passed')
        return

    if opts.log_dir is not None and opts.log_prefix is None:
        opts.log_prefix = os.path.abspath(os.path.join(
            opts.log_dir,
            time.strftime('mjlog-%Y%m%d-%H%M%S', time.localtime())))

    if opts.log_prefix:
        print 'Saving logs to %s.*' % opts.log_prefix
        txtname = opts.log_prefix + '.log'
        try:
            os.makedirs(os.path.dirname(txtname))
        except OSError:
            pass

        txtlog = logging.FileHandler(txtname, encoding='utf-8')
        txtlog.setFormatter(logging.Formatter(
            "%(asctime)s.%(msecs).3d [%(levelname).1s] %(name)s: %(message)s",
            datefmt="%F %T"))
        txtlog.setLevel(logging.DEBUG)
        root.addHandler(txtlog)


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
                      help='Exit immediately after loading program')
    parser.add_option('--addr', metavar='IP',
                      help='Client address (default autodiscover)')
    parser.add_option('--external-video', action='store_true',
                      help='Use external process for video playing')
    parser.add_option('--log-prefix', default=None,
                      help='Write logfiles with specified prefix '
                      '(must include path, default base on log-dir and '
                      'current timestamp)')
    parser.add_option('-L', '--log-dir', default='./mjmech-data',
                      help='Write logfiles using autogenerated names in '
                      'this directory. Empty to disable. Default "%default".')
    parser.add_option('-R', '--restore-state', default=None,
                      help='Restore UI state on startup. Default LOGDIR/' +
                      ControlInterface.UI_STATE_SAVEFILE)

    opts, args = parser.parse_args()
    if len(args) and opts.addr is None:
        opts.addr = args.pop(0)
    if args:
        parser.error('Invalid number of args')

    main(opts)
