#!/usr/bin/python
import collections
import cStringIO as StringIO
import errno
import functools
import json
import logging
import optparse
import os
import signal
import socket
import sys
import time
import traceback

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

# Possible locations for logdir. If any of the paths exist, will use this one.
# else, will use the last entry.
LOG_DIR_LOCATIONS = [
    '~/.mjmech-data',
    '~/mjmech-data',
    './mjmech-data' ]


# based on example at:
# http://bazaar.launchpad.net/~jderose/+junk/gst-examples/view/head:/video-player-1.0
# docs:
# http://pygstdocs.berlios.de/pygst-reference/gst-class-reference.html

g_main_loop = None
def wrap_event(callback):
    """Wrap event callback so the app exit if it crashes"""
    def wrapped(*args, **kwargs):
        try:
            return callback(*args, **kwargs)
        except BaseException as e:
            logging.error("Callback %r crashed:", callback)
            logging.error(" %s %s" % (e.__class__.__name__, e))
            for line in traceback.format_exc().split('\n'):
                logging.error('| %s', line)
            if g_main_loop is not None:
                g_main_loop.call_soon_threadsafe(
                    g_main_loop.stop)
            Gtk.main_quit()
            raise
    return wrapped

def add_pair(a, b):
    return (a[0] + b[0], a[1] + b[1])

RIPPLE_COMMAND = {
    'type': 'ripple',
    'translate_x_mm_s': 0,
    'translate_y_mm_s': 0,
    'rotate_deg_s': 0,
    }

IDLE_COMMAND = {
    'type': 'idle'
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
        self.sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

        self.sock.setblocking(0)
        self.logger.info('Connecting to %s:%d' % self.addr)
        self.sock.connect(self.addr)

        # Wallclock and pipeline time
        self.last_video_wall_time = None
        self.last_video_pp_time = None

        self.seq = 0
        self.control_dict = dict(
            boot_time = time.time(),

            video_port=self.VIDEO_PORT,
            # Not set: turret - pair of (x, y) degrees
            # Not set: gait - a dict based off IDLE/RIPPLE_COMMAND

            # 0 = off; 1 = on
            laser_on = 0,
            agitator_on = 0,
            green_led_on = 0,

            # pwm values in 0..1 range
            agitator_pwm = 0.5,
            fire_motor_pwm = 0.75,
            # fire command duration (seconds)
            fire_duration = 0.5,

            # fire a shot every time this changes; should only ever increase
            fire_cmd_count = 0,
            )

        # If True, some gait-related key is being held.
        self.key_gait_active = False

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
            if len(abs_axis) >= 3:
                return 1

            return -1

        enumerator = joystick.JoystickEnumerator(metric)
        joysticks = enumerator.joysticks()
        if len(joysticks):
            self.joystick = joysticks[0]
            self.joystick_task = Task(self._read_joystick())
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

    def select_axes(self):
        complete = self.joystick.get_features(linux_input.EV.ABS)

        # If we have all of X, Y, and RX, RY, then the axes are
        # probably mapped correctly and we can return what we expect.
        if (linux_input.ABS.X in complete and
            linux_input.ABS.Y in complete and
            linux_input.ABS.RX in complete and
            linux_input.ABS.RY in complete):
            return [linux_input.ABS.X, linux_input.ABS.Y, linux_input.ABS.RX]

        # Otherwise, just return the first three axes on the hope that
        # this is meaningful.
        return complete[0:3]

    @wrap_event
    def _read_joystick(self):
        axes = self.select_axes()
        print "Selected joystick axes:", axes

        while True:
            ev = yield From(self.joystick.read())

            if ev.ev_type != linux_input.EV.ABS:
                continue

            dx = self.joystick.absinfo(axes[0]).scaled()
            dy = self.joystick.absinfo(axes[1]).scaled()
            dr = self.joystick.absinfo(axes[2]).scaled()

            if abs(dx) < 0.2 and abs(dy) < 0.2 and abs(dr) < 0.2:
                if self.control_dict['gait'] is not None:
                    self.control_dict['gait'] = IDLE_COMMAND
            else:
                gait = RIPPLE_COMMAND.copy()
                gait['translate_x_mm_s'] = dx * 50
                gait['translate_y_mm_s'] = -dy * 200
                gait['rotate_deg_s'] = dr * 50
                self.control_dict['gait'] = gait

    @wrap_event
    def _on_send_timer(self):
        # Raise exception if any of tasks terminate
        if self.joystick_task and self.joystick_task.done():
            #self.joystick_task.result()
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
            gait = RIPPLE_COMMAND.copy()
            gait['translate_x_mm_s'] = dx * 50
            gait['translate_y_mm_s'] = dy * 100
            gait['rotate_deg_s'] = dr * 30
            self.control_dict['gait'] = gait
            self.key_gait_active = True
        elif name == 'h':
            self._print_help()
        elif name == 'l':
            self.control_dict['laser_on'] ^= 1
            self.logger.info('Laser set to %d',
                             self.control_dict['laser_on'])
        elif name == 'm':
            self.control_dict['agitator_on'] ^= 1
            self.logger.info('Agitator set to %d (pwm %.3f)',
                             self.control_dict['agitator_on'],
                             self.control_dict['agitator_pwm'])
        elif name == 'G':
            self.control_dict['green_led_on'] ^= 1
            self.logger.info('Green LED set to %d',
                             self.control_dict['green_led_on'])
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
        if self.key_gait_active:
            self.key_gait_active = False
            if self.control_dict['gait'] is not None:
                self.control_dict['gait'] = IDLE_COMMAND
            self._send_control()

    def _print_help(self):
        print 'Keys:'
        print '  w/s, a/d - move'
        print '  q/e      - rotate'
        print '  l        - laser on/off'
        print '  m        - agitator on/off'
        print '  G        - green LED on/off'
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
    # How often to print framerate info
    VIDEO_INFO_INTERVAL = 30.0

    # If True, will dump info for all pads when framerate expires
    DUMP_PAD_INFO_BY_TIMER = False

    # If True, will crash app if video stops
    CRASH_ON_VIDEO_STOP = True

    # Is camera upside down?
    CAMERA_ROTATE = False

    # UDP sources will generate a warning when that many seconds without
    # packets pass
    RTP_UDP_WARN_TIMEOUT = 5.0
    # RTCP has quite low packet rate (once every 5 seconds or so), so timeout is
    # much longer
    RTCP_UDP_WARN_TIMEOUT = 30.0

    def __init__(self, host, port, video_log=None):
        self.host = host
        self.port = port
        self.logger = logging.getLogger('video')
        self.stats_logger = self.logger.getChild('stats')

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

        # info for detectors to write to.
        self.detector_stats = dict()

        # Create 'identity' elements which will notify us of all passing buffers
        self.detector_decoded = self.make_element(
            "identity", name="detector_decoded", silent=False)
        self.detector_decoded.connect(
            "handoff", self._on_detector_handoff,
            self.detector_stats.setdefault("decoded", self._DetectorStats()))

        self.detector_raw = self.make_element(
            "identity", name="detector_raw", silent=False)
        self.detector_raw.connect(
            "handoff", self._on_detector_handoff,
            self.detector_stats.setdefault("raw", self._DetectorStats()))

        self.detector_udp_rtp = self.make_element(
            "identity", name="detector_udp_rtp", silent=False)
        self.detector_udp_rtp.connect(
            "handoff", self._on_detector_handoff,
            self.detector_stats.setdefault("udp_rtp", self._DetectorStats()))

        self.rtpbin = self.make_element(
            "rtpbin",
            do_retransmission=True,
            # notify on individual packet losses(TODO mafanasyev: hook this)
            do_lost=True,
            # remove pad when client disappears
            autoremove=True,
            # TODO mafanasyev: try settings below, maybe they will help
            #use_pipeline_clock=True,
            #buffer_mode=2,   # RTP_JITTER_BUFFER_MODE_BUFFER
            )
        self.rtpbin_last_pad = None

        caps = Gst.Caps.from_string(
            "application/x-rtp,media=(string)video,clock-rate=(int)90000,"
            "encoding-name=(string)H264")
        rtp_src = self.make_element(
            "udpsrc", caps=caps, port=self.port,
            name="rtp_src", timeout=long(self.RTP_UDP_WARN_TIMEOUT * 1.0e9))
        self.link_pads(rtp_src, None, self.detector_udp_rtp, None)
        self.link_pads(self.detector_udp_rtp, None,
                       self.rtpbin, "recv_rtp_sink_0")

        rtcp_src = self.make_element(
            "udpsrc", port=self.port + 1,
            name="rtcp_src", timeout=long(self.RTCP_UDP_WARN_TIMEOUT * 1.0e9))
        self.link_pads(rtcp_src, None, self.rtpbin, "recv_rtcp_sink_0")

        rtcp_sink = self.make_element("udpsink", host=self.host,
                                      port=self.port + 2,
                                      sync=False, async=False)
        self.link_pads(self.rtpbin, "send_rtcp_src_0", rtcp_sink, None)

        self.info_overlay = self.make_element(
            "rsvgoverlay",
            fit_to_frame=True)

        # We have a problem: info_overlay only updates when there are video
        # frames. Thus, if video stops, so does OSD. This is inconvinient.
        # There is 'videorate' element which supposedly can repeat frames to
        # maintain framerate, but it is designed for offline processing, and
        # will not output anything if there is no input.
        #
        # How can we fix this?
        #  - fix videorate.c to add timer, or write our own component from
        #    scratch
        #  - use 'input-selector' to select 'videotestsrc' if there is no data
        #    for a while.
        #  - use 'appsrc'/'appsink' to do duplicate frames without overhead of
        #    full component.

        decode_elements = [
            # Add a queue just in case
            self.make_element('queue', name='queue_decode'),
            self.make_element("rtph264depay"),
            self.detector_raw,
            self.make_element("h264parse"),
            self.make_element("tee"),
        ]
        play_elements = decode_elements + [
            self.make_element('queue', name='queue_play'),
            self.make_element("avdec_h264"),
            self.make_element("videoconvert"),
            ]
        if self.CAMERA_ROTATE:
            play_elements.append(self.make_element(
                    "videoflip", method="clockwise"))
        self.imagesink = self.make_element("xvimagesink")
        play_elements += [
            self.make_element("timeoverlay", shaded_background=True,
                              font_desc="8",
                              valignment="bottom", halignment="right"),
            self.detector_decoded,
            self.info_overlay,
            self.make_element("videoconvert"),
            self.imagesink,
            ]

        self.link_list_of_pads(play_elements)
        self.play_elements = play_elements
        self.rtpbin.connect("pad-added", self._on_new_rtpbin_pad)
        self.rtpbin.connect("pad-removed", self._on_removed_rtpbin_pad)

        if False:
            # Connect something to the old queue, so pipeline can start.
            self.link_list_of_pads(
                [self.make_element("fakesrc", is_live=True),
                 self.make_element("capsfilter", caps=caps),
                 self.play_elements[0]])

            # Create new play elements queue which is empty.
            self.play_elements = [
                self.detector_decoded,
                self.make_element("fakesink")
                ]
            self.link_list_of_pads(self.play_elements)


        if video_log is not None:
            self.logger.info('Recording video to %r' % video_log)
            save_elements = [
                decode_elements[-1],
                self.make_element('queue', name='queue_save'),
                self.make_element('matroskamux'),
                self.make_element('filesink', location=video_log)
            ]
            self.link_list_of_pads(save_elements)

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_any_message)

        # Enable 'sync' messages which are not thread-safe. We should do as
        # little as possible there to avoid random crashes.
        bus.enable_sync_message_emission()
        bus.connect('sync-message::element', self._on_sync_message)

        # dropped packet info. tuple (drop-count, drop-time)
        self.qos_dropped_info = dict()

        # Start periodic video information timer
        GLib.timeout_add(int(self.VIDEO_INFO_INTERVAL * 1000),
                         self._on_video_info_timer)

        # Callback functions
        self.on_video_click_1 = None
        self.on_key_press = None
        self.on_key_release = None
        self.on_got_video = None

    def make_element(self, etype, name=None, **kwargs):
        elt = Gst.ElementFactory.make(etype, name)
        assert elt, 'Failed to make element %r of type %r' % (name, etype)
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
            elt1.get_name(), pad1, elt2.get_name(), pad2)

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
        self.logger.info('Got new rtpbin pad: %r', name)
        if name.startswith('recv_rtp_src'):
            if self.rtpbin_last_pad is not None:
                #ok = self.rtpbin.unlink(
                #    self.rtpbin_last_pad, self.play_elements[0], None)
                # Since we do not know dest pad, unlink all.
                ok = self.rtpbin.unlink(self.play_elements[0])
                self.logger.info('Unlinking old pad: %r', ok)
            self.rtpbin_last_pad = name
            self.link_pads(self.rtpbin, name, self.play_elements[0], None)

    @wrap_event
    def _on_removed_rtpbin_pad(self, source, pad):
        name = pad.get_name()
        self.logger.info('Rtpbin pad got removed: %r', name)

    @wrap_event
    def _on_sync_message(self, bus, msg):
        # This is run in internal thread. Try to do as little as possible there.
        struct_name = msg.get_structure().get_name()
        if struct_name == 'prepare-window-handle':
            self.logger.debug('Embedding video window')
            msg.src.set_window_handle(self.xid)
        return True

    @wrap_event
    def _on_any_message(self, bus, msg):
        if msg.type == Gst.MessageType.STATE_CHANGED:
            old, new, pending = msg.parse_state_changed()
            # Only log this for some elements
            if msg.src == self.play_elements[-1]:
                # enum has value_name='GST_STATE_PAUSED', value_nick='playing'
                msg_text = '%s->%s' % (old.value_nick, new.value_nick)
                if pending != Gst.State.VOID_PENDING:
                    msg_text += ' (pending %s)' % pending.value_nick
                self.logger.debug("GST State changed for %s: %s",
                                 msg.src.get_name(), msg_text)
        elif msg.type == Gst.MessageType.QOS:
            sname = msg.src.get_name()
            live, t_running, t_stream, t_timestamp, drop_dur = msg.parse_qos()

            if sname not in ['rsvgoverlay0', 'xvimagesink0', 'videoconvert0']:
                st_format, st_processed, st_dropped = msg.parse_qos_stats()
                v_jitter, v_proportion, v_quality = msg.parse_qos_values()

                # Only print stats from unusual sources
                self.logger.debug(
                    'QOS drop info from %s: live=%r, running_time=%.4f, '
                    'stream_time=%.4f, timestamp=%.4f, drop_duration=%.4f',
                    sname, live, t_running / 1.e9, t_stream / 1.e9,
                    t_timestamp / 1.e9, drop_dur / 1.e9)
                self.logger.debug(
                    'QOS stats for %s: (%s) processed=%d, dropped=%d',
                    sname, st_format.value_nick, st_processed, st_dropped)
                self.logger.debug(
                    'QOS values for %s: jitter=%.4f proportion=%.5f quality=%d',
                    sname, v_jitter, v_proportion / 1.e9, v_quality)

            if drop_dur not in [0, Gst.CLOCK_TIME_NONE]:
                old = self.qos_dropped_info.get(sname, (0, 0))
                self.qos_dropped_info[sname] = (old[0] + 1,
                                                old[1] + drop_dur / 1.e9)

        elif msg.type == Gst.MessageType.STREAM_STATUS:
            status, owner = msg.parse_stream_status()
            if False:
                # Boring.
                self.logger.debug(
                    "Stream status changed for %r: %s for owner %r",
                    msg.src.get_name(),
                    status.value_name, owner.get_name())
            # TODO mafanasyev: wait for 'rtpjitterbuffer0' element, then
            # record it's pointer so one can query 'stats' with usefull stuff
            # like udp packet loss/duplication.

        elif msg.type == Gst.MessageType.STREAM_START:
            has_group, group_id = msg.parse_group_id()
            if not has_group:
                group_id = None
            self.logger.info("Stream started (source %s, group %r)",
                             msg.src.get_name(), group_id)
            if self.on_got_video:
                self.on_got_video()

        elif msg.type == Gst.MessageType.ERROR:
            err, debug = msg.parse_error()
            self.logger.error("GstError from %s: %s", msg.src.get_name(), err)
            if debug:
                for line in debug.split('\n'):
                    self.logger.info('| %s' % line)

        elif msg.type == Gst.MessageType.WARNING:
            err, debug = msg.parse_warning()
            self.logger.error("GstWarning from %s: %s", msg.src.get_name(), err)
            if debug:
                for line in debug.split('\n'):
                    self.logger.info('| %s' % line)

        elif msg.type == Gst.MessageType.ELEMENT:
            # Element-specific message
            mstruct = msg.get_structure()
            struct_name = mstruct.get_name()
            if struct_name == 'prepare-window-handle':
                pass    # We handle this as sync message
            elif struct_name == 'GstUDPSrcTimeout':
                self.logger.warn('No data on UDP sink %r '
                                 '(port %d, timeout %.3f sec)',
                                 msg.src.get_name(),
                                 msg.src.get_property('port'),
                                 mstruct.get_value('timeout') / 1.e9)
            else:
                self.logger.debug("Element %r says: %s" % (
                        msg.src.get_name(), mstruct.to_string()))
        elif msg.type in [Gst.MessageType.ASYNC_DONE,
                          Gst.MessageType.NEW_CLOCK]:
            pass   # internal, boring
        else:
            # Unknown pre-defined message
            self.logger.debug("Unknown system message from %s: %s" % (
                    msg.src.get_name(),  msg.type.value_names))

        return True

    def _evt_get_video_coord(self, evt):
        # Get size of window
        alloc = self.drawingarea.get_allocation()
        wnd_size = (alloc.width, alloc.height)
        #wnd_size = (self.imagesink.get_property('window-width'),
        #            self.imagesink.get_property('window-height'))

        # Get original size of video stream
        caps = self.imagesink.sinkpad.get_current_caps()
        if caps is None:
            return None

        # Assume these are simple caps with a single struct.
        struct = caps.get_structure(0)
        video_size = (struct.get_int('width')[1], struct.get_int('height')[1])

        # Calculate image position in (-1..1) range (taking in the account
        # that video is scaled, but aspect ratio is preserved)
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


    class _DetectorStats(object):
        """Detector stats object, filled by detectors' handoff signal."""
        def __init__(self):
            self.clear()

        def clear(self):
            self.count = 0
            self.duration = 0.0
            self.size = 0
            self.last_dts = 0
            self.last_pts = 0

        def to_str_dt(self, dt, level=0):
            tags = ['%.2f FPS' % (self.count / dt)]
            if level >= 2:
                tags.append('%.1f%% miss' % ((dt - self.duration) / dt))
            if level >= 1:
                if self.size > (1024 * 1024 * 50):
                    tags.append('%.1fMB/s' % (self.size / 1024.0 / 1024.0 / dt))
                else:
                    tags.append('%.1fkB/s' % (self.size / 1024.0 / dt))
            return ', '.join(tags)

    @wrap_event
    def _on_detector_handoff(self, sender, gbuffer, statrec):
        # This is a callback for identity object which is called after each
        # frame. It runs from video thread and should do as little as possible.
        statrec.count += 1
        statrec.size += gbuffer.get_size()
        if gbuffer.dts != Gst.CLOCK_TIME_NONE:
            statrec.last_dts = gbuffer.dts / 1.e9
        if gbuffer.pts != Gst.CLOCK_TIME_NONE:
            statrec.last_pts = gbuffer.pts / 1.e9
        if gbuffer.duration != Gst.CLOCK_TIME_NONE:
            statrec.duration += gbuffer.duration / 1.e9

        # GstBuffer's documentation also mentions flags, but I do not know
        # how to access them.
        #statrec['last_flags'] = int(gbuffer.flags)


    @wrap_event
    def _on_da_release(self, src, evt):
        assert src == self.window, src
        if self.on_key_release:
            self.on_key_release(evt)
        return True

    @wrap_event
    def _on_video_info_timer(self):
        # Assume loop has no large delays
        dt = self.VIDEO_INFO_INTERVAL * 1.0

        # print out drop info from QoS mesages
        drop_tags = list()
        for name, (count, duration) in sorted(self.qos_dropped_info.items()):
            drop_tags.append('%s: %d (%.3fs)' % (name, count, duration))
        if drop_tags:
            self.stats_logger.info('Frames dropped: %s', '; '.join(drop_tags))
        self.qos_dropped_info.clear()


        decoded_frames = self.detector_stats['decoded'].count

        # print out statistics from QoS messages
        det_tags = list()
        for name, value in sorted(self.detector_stats.items()):
            det_tags.append(
                '%s: %s' % (name, value.to_str_dt(dt, level=1)))
            value.clear()

        self.stats_logger.info(
            'Video stats: %s', '; '.join(det_tags) or 'No frames detected')

        if not decoded_frames and self.CRASH_ON_VIDEO_STOP:
            self._dump_pad_info()
            assert False, 'Video stream broken'

        if self.DUMP_PAD_INFO_BY_TIMER:
            self._dump_pad_info()

        # Keep timer going
        return True

    def _dump_pad_info(self):
        now = self.pipeline.get_clock().get_internal_time() / 1.e9
        self.logger.debug('Dumping pad info at time %f' % now)
        to_check = [ ('', self.pipeline) ]

        while to_check:
            prefix, elt = to_check.pop(0)

            # Extract type without __main__ prefix
            typename = type(elt).__name__.split('.', 1)[-1]
            if typename == 'GstQueue':
                stats = (
                    elt.get_property('current-level-time') / 1.e9,
                    elt.get_property('current-level-buffers'),
                    elt.get_property('current-level-bytes')
                    )
                if sum(stats) != 0:
                    self.logger.debug(
                        '  Queue %r level: time=%.5f, buffers=%d, bytes=%d',
                        elt.get_name(), *stats)

            if hasattr(elt, 'children'):
                # GstBin or subclass
                if elt == self.pipeline:
                    ename = ''
                else:
                    ename = prefix + elt.get_name() + '.'
                to_check += [(ename, child)
                             for child in elt.children]
                self.logger.debug(' Bin %r', prefix + elt.get_name())

            for pad in reversed(elt.pads):
                tags = list()

                if pad.is_blocked(): tags.append('BLOCKED')
                if pad.is_blocking(): tags.append('BLOCKING')
                if not (pad.is_linked() or tags):
                    # It is a bad idea to query unlinked elements
                    continue

                ok, pos_bytes = pad.query_position(Gst.Format.BYTES)
                if ok and pos_bytes != 0:
                    tags.append('pos_bytes=%d' % pos_bytes)

                ok, pos_time = pad.query_position(Gst.Format.TIME)
                if ok and pos_time != 0:
                    if pos_time:
                        tags.append('pos_time=%.6f' % (pos_time / 1.e9))
                    else:
                        tags.append('pos_time=0')

                if tags:
                    self.logger.debug(
                        '  Pad %s%s.%s: %s',
                        prefix, elt.get_name(), pad.get_name(), ', '.join(tags))


    def set_svg_overlay(self, data):
        self.info_overlay.set_property("data", data)


@wrap_event
def _sigint_handler():
    # wrap_event decorator will make sure the exception stops event loop.
    raise Exception('Got SIGINT')

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

    if opts.log_dir is None:
        for pdir in LOG_DIR_LOCATIONS:
            opts.log_dir = os.path.abspath(os.path.expanduser(pdir))
            if os.path.exists(opts.log_dir):
                break
        logging.info('Auto-detected logdir as %r', opts.log_dir)

    if opts.check:
        logging.info('Check passed')
        return

    if opts.log_dir != '' and opts.log_prefix is None:
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

    global g_main_loop
    g_main_loop = asyncio.get_event_loop()
    g_main_loop.add_signal_handler(signal.SIGINT, _sigint_handler)

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
    parser.add_option('-L', '--log-dir',
                      help='Write logfiles using autogenerated names in '
                      'this directory. Empty to disable. Default autodetect.')
    parser.add_option('-R', '--restore-state', default=None,
                      help='Restore UI state on startup. Default LOGDIR/' +
                      ControlInterface.UI_STATE_SAVEFILE)

    opts, args = parser.parse_args()
    if len(args) and opts.addr is None:
        opts.addr = args.pop(0)
    if args:
        parser.error('Invalid number of args')

    main(opts)
