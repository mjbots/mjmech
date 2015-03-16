#!/usr/bin/env python
import collections
import logging
import os
import sys
import time

# IMPORT NOTE:
#  this file may be later re-written in C, if we need more performance
#  so keep to C-like API, and specifically do not use trollius/asyncio
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
from gi.repository import GObject, GLib
from gi.repository import GdkX11, GstVideo, Gtk, Gdk

from vui_helpers import wrap_event, asyncio_misc_init, g_quit_handlers

# based on example at:
# http://bazaar.launchpad.net/~jderose/+junk/gst-examples/view/head:/video-player-1.0
# docs:
# http://pygstdocs.berlios.de/pygst-reference/gst-class-reference.html
# another example:
# https://github.com/RichardWithnell/evaluation_tools/blob/fb50facfc585f174f8ece01db85118b22b2b77e8/applications.py

class VideoWindow(object):
    # How often to print framerate info
    VIDEO_INFO_INTERVAL = 30.0

    # If True, will dump info for all pads when framerate expires
    DUMP_PAD_INFO_BY_TIMER = False

    # If True, will crash app if video stops
    CRASH_ON_VIDEO_STOP = False

    # Is camera upside down?
    CAMERA_ROTATE = False

    # Dump extra events info
    DUMP_EXTRA_EVENTS = False

    # Dump extra stats
    DUMP_EXTRA_STATS = False

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

        # Either None, or a button number
        self._mouse_down_info = None
        # If True, mouse move suppression timer is active
        self._mouse_move_timer_pending = False
        # If not None, suppressed mouse move position
        self._mouse_move_suppressed_pos = None

        self.drawingarea = Gtk.DrawingArea()
        self.drawingarea.add_events(Gdk.EventMask.BUTTON_PRESS_MASK |
                                    Gdk.EventMask.BUTTON_RELEASE_MASK |
                                    Gdk.EventMask.POINTER_MOTION_MASK
                                    )
        self.drawingarea.connect("button-press-event", self._on_da_mouse_press)
        self.drawingarea.connect("button-release-event",
                                 self._on_da_mouse_release)
        self.drawingarea.connect("motion-notify-event", self._on_da_move)
        self.window.connect("key-press-event", self._on_da_key_press)
        self.window.connect("key-release-event", self._on_da_key_release)
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
        self.decoded_stats = self.detector_stats.setdefault(
            "decoded", self._DetectorStats(show_size=False))
        self.detector_decoded.connect(
            "handoff", self._on_detector_handoff, self.decoded_stats)

        self.detector_raw = self.make_element(
            "identity", name="detector_raw", silent=False)
        self.detector_raw.connect(
            "handoff", self._on_detector_handoff,
            self.detector_stats.setdefault(
                "raw", self._DetectorStats(show_size=False)))

        self.detector_udp_rtp = self.make_element(
            "identity", name="detector_udp_rtp", silent=False)
        self.detector_udp_rtp.connect(
            "handoff", self._on_detector_handoff,
            self.detector_stats.setdefault("udp_rtp", self._DetectorStats()))

        # Info for other callbacks
        self.extra_stats = collections.defaultdict(int)

        self.last_jitterbuffer = None
        self.last_jitterbuffer_stats = dict()

        self.rtpbin = self.make_element(
            "rtpbin",
            do_retransmission=True,
            # notify on individual packet losses(TODO mafanasyev: hook this)
            do_lost=True,
            # remove pad when client disappears
            autoremove=True,
            # Maximum latency, ms (default 200)
            latency=200,

            # TODO mafanasyev: try settings below, maybe they will help
            #use_pipeline_clock=True,
            #buffer_mode=2,   # RTP_JITTER_BUFFER_MODE_BUFFER
            )
        self.rtpbin_last_pad = None

        if self.DUMP_EXTRA_STATS:
            self.rtpbin_last_rtpstats = dict(key=None)

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
        self.rtpbin.connect("on-ssrc-active", self._on_rtpbin_ssrc_active)
        # TODO mafanasyev: figure out why this breaks video, then set various
        # properties in a handler
        #self.rtpbin.connect("new-jitterbuffer",
        #                    self._on_new_rtpbin_jitterbuffer)

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
        self.on_video_mouse_click = None
        self.on_video_mouse_move = None
        self.on_video_mouse_release = None
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

    def get_video_pts(self):
        """Return video PTS (presentation time stamp) in seconds. This
        corresponds to timestamp in .mkv file and may be used for matching
        osd to video.
        """
        return self.decoded_stats.last_pts

    @wrap_event
    def _on_new_rtpbin_pad(self, source, pad):
        name = pad.get_name()
        self.logger.debug('Got new rtpbin pad: %r', name)
        if name.startswith('recv_rtp_src'):
            if self.rtpbin_last_pad is not None:
                #ok = self.rtpbin.unlink(
                #    self.rtpbin_last_pad, self.play_elements[0], None)
                # Since we do not know dest pad, unlink all.
                ok = self.rtpbin.unlink(self.play_elements[0])
                self.logger.debug('Unlinking old pad: %r', ok)
            self.rtpbin_last_pad = name
            self.link_pads(self.rtpbin, name, self.play_elements[0], None)
            pad.add_probe(Gst.PadProbeType.EVENT_DOWNSTREAM,
                          self._on_rtpbin_downstream_event)

    @wrap_event
    def _on_rtpbin_downstream_event(self, pad, info):
        event = info.get_event()
        if event.type != Gst.EventType.CUSTOM_DOWNSTREAM:
            if self.DUMP_EXTRA_EVENTS:
                name = event.type.value_name
                self.logger.debug('Rtpbin downstream basic event: %s', name)
            return Gst.PadProbeReturn.PASS

        name = event.get_structure().get_name()
        if name == 'GstRTPPacketLost':
            # Packet lost, retries (if any) failed
            if self.DUMP_EXTRA_STATS:
                self.extra_stats['loss-final'] += 1
            if self.DUMP_EXTRA_EVENTS:
                self.logger.debug('Rtpbin downstream packet loss')
        else:
            self.logger.debug('Rtpbin downstream custom event: %s', name)

        return Gst.PadProbeReturn.PASS

    @wrap_event
    def _on_removed_rtpbin_pad(self, source, pad):
        name = pad.get_name()
        self.logger.debug('Rtpbin pad got removed: %r', name)
        # Release jitterbiffer value just in case
        self.last_jitterbuffer = None

    @wrap_event
    def _on_rtpbin_ssrc_active(self, rtpbin, session_id, ssrc):
        # This is invoked every time RTCP is sent
        assert rtpbin == self.rtpbin

        if not self.DUMP_EXTRA_STATS:
            return

        # Get RtpSession object
        session = rtpbin.emit("get-internal-session", session_id)
        if session is None:
            self.logger.warn("Could not get internal session for SSRC")
            return
        # Get RTPSource object (see rtpsource.c)
        source = session.emit("get-source-by-ssrc", ssrc)
        if session is None:
            self.logger.warn("Could not get internal source for SSRC")
            return
        # Get RTPSource::stats object (see rtpsource.c)
        stats = source.get_property('stats')
        if stats is None:
            self.logger.warn("Could not get internal stats for SSRC")
            return

        # Check if we have previous stats object
        if self.rtpbin_last_rtpstats['key'] != (session_id, ssrc):
            self.logger.debug('Selecting SSRC %r', (session_id, ssrc))
            # Wipe previous stats
            self.rtpbin_last_rtpstats = dict(key = (session_id, ssrc))
        prev = self.rtpbin_last_rtpstats

        # Unfortunately, 'rb-round-trip' is always zero.. it would be nice
        # to log this value.

        # Get RTCP packet loss metric.. it is somewhat useless as it includes
        # duplicates and losses in a single number.
        packets_lost = stats.get_int('sent-rb-packetslost')[1]
        dp = packets_lost - prev.get('packets_lost', 0)
        if dp > 0:
            self.extra_stats['loss-rtcp'] += dp
        elif dp < 0:
            # Duplicate packet. Unfortunately, there is no way to get both
            # dups and losses separately
            self.extra_stats['dupe-rtcp'] += -dp

        prev.update(packets_lost=packets_lost)

        if self.DUMP_EXTRA_EVENTS:
            self.logger.debug('SSRC active: sess=%r, ssrc=%r, %r',
                          session_id, ssrc, stats.to_string())


    @wrap_event
    def _on_new_rtpbin_jitterbuffer(self, source, jbuffer, session, ssrc):
        # TODO mafanasyev: this is currently disabled now
        self.logger.debug('Rtpbin made jitterbuffer %r for session %r ssrc %r',
                          jbuffer, session, ssrc)
        # TODO mafanasyev: configure jitterbuffer:
        #  set 'rtx-delay' to ask for retransmittions earlier
        #  set 'rtx-delay-reorder' to handle reordering faster
        #  set 'rtx-retry-timeout' to retry more

    @wrap_event
    def _on_sync_message(self, bus, msg):
        # This is run in internal thread. Try to do as little as possible there.
        struct_name = msg.get_structure().get_name()
        if struct_name == 'prepare-window-handle':
            if self.DUMP_EXTRA_EVENTS:
                self.logger.debug('Embedding video window')
            msg.src.set_window_handle(self.xid)
        return True

    @wrap_event
    def _on_any_message(self, bus, msg):
        if msg.type == Gst.MessageType.STATE_CHANGED:
            old, new, pending = msg.parse_state_changed()
            # Only log this for some elements
            if msg.src == self.play_elements[-1] and self.DUMP_EXTRA_EVENTS:
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
            if self.DUMP_EXTRA_EVENTS:
                # Boring.
                self.logger.debug(
                    "Stream status changed for %r: %s for owner %r",
                    msg.src.get_name(),
                    status.value_name, owner.get_name())
            if owner.get_name() == 'rtpjitterbuffer0':
                self.last_jitterbuffer = owner
                self.last_jitterbuffer_stats = dict()

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
            self.logger.warn("GstWarning from %s: %s", msg.src.get_name(), err)
            if debug:
                for line in debug.split('\n'):
                    self.logger.info('| %s' % line)

        elif msg.type == Gst.MessageType.TAG:
            tags = msg.parse_tag()
            self.logger.debug("Tag from %s: %s",
                              msg.src.get_name(), tags.to_string())

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
            elif struct_name == 'application/x-rtp-source-sdes':
                self.logger.debug("Source appears: tool %r at %r",
                                  mstruct.get_value('tool'),
                                  mstruct.get_value('cname'))
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

    def get_video_window_size(self):
        """Get visible size of video window. Returns (width, height) tuple"""
        alloc = self.drawingarea.get_allocation()
        return (alloc.width, alloc.height)

    def get_video_size(self):
        """Get video size. Returns (width, height) tuple, or None
        if video is not ready yet.
        """
        # Get original size of video stream
        caps = self.imagesink.sinkpad.get_current_caps()
        if caps is None:
            return None

        # Assume these are simple caps with a single struct.
        struct = caps.get_structure(0)
        return (struct.get_int('width')[1], struct.get_int('height')[1])


    def _evt_get_video_coord(self, evt):
        """Given a mouse-click event from the video window, convert it to
        video coordinates scaled into (0-1) range.
        Returns None for click outside of window, else 2-tuple.
        """
        # Get size of window
        wnd_size = self.get_video_window_size()
        video_size = self.get_video_size()
        if video_size is None:
            return None

        # imagesink centers the video while preserving aspect ratio. Reproduce
        # the scaling logic.
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
        if self._mouse_down_info is None:
            return True

        # Mouse moved while mouse button is held
        rel_pos = self._evt_get_video_coord(evt)
        if not rel_pos:
            return True

        if self._mouse_move_timer_pending:
            # Suppression timer active, record position
            self._mouse_move_suppressed_pos = rel_pos
        else:
            # Have not reported mouse moves recently, start suppression timer
            if self.on_video_mouse_move:
                self.on_video_mouse_move(rel_pos, self._mouse_down_info)
            # Set timer to ratelimit events (every 50 mS)
            self._mouse_move_timer_pending = True
            GLib.timeout_add(50, self._on_da_move_timeout)

        return True

    @wrap_event
    def _on_da_move_timeout(self):
        if self._mouse_move_suppressed_pos:
            # We had suppressed position.
            if (self._mouse_down_info is not None
                ) and self.on_video_mouse_move:
                self.on_video_mouse_move(self._mouse_move_suppressed_pos,
                                         self._mouse_down_info)
            self._mouse_move_suppressed_pos = None
            # Keep timer going
            return True

        # No mouse motion. Stop timer.
        self._mouse_move_timer_pending = False
        return False

    @wrap_event
    def _on_da_mouse_press(self, src, evt):
        assert src == self.drawingarea, src
        rel_pos = self._evt_get_video_coord(evt)
        if rel_pos is None:
            self.logger.info(
                'Video click outside of image at wpt=(%d,%d) button=%d '
                'state=%d', evt.x, evt.y, evt.button, evt.state)
            return True

        self._mouse_down_info = evt.button
        #self.logger.debug(
        #    'Video click at wpt=(%d,%d) rel=(%.3f,%.3f) button=%d state=%d',
        #    evt.x, evt.y, rel_pos[0], rel_pos[1], evt.button, evt.state)
        if self.on_video_mouse_click:
            self.on_video_mouse_click(rel_pos, button=evt.button)
        return True

    @wrap_event
    def _on_da_mouse_release(self, src, evt):
        assert src == self.drawingarea, src
        self._mouse_down_info = None
        if self.on_video_mouse_release:
            self.on_video_mouse_release(evt.button)
        return True

    @wrap_event
    def _on_da_key_press(self, src, evt):
        assert src == self.window, src
        if self.on_key_press:
            # Parse out the keys
            base_name = Gdk.keyval_name(evt.keyval)
            modifiers = ''
            MT = Gdk.ModifierType
            if evt.state & MT.CONTROL_MASK: modifiers += 'C-'
            if evt.state & MT.SHIFT_MASK: modifiers += 'S-'
            if evt.state & MT.MOD1_MASK: modifiers += 'M-'
            self.on_key_press(base_name, modifiers)
        return True


    class _DetectorStats(object):
        """Detector stats object, filled by detectors' handoff signal."""
        def __init__(self, show_size=True):
            self._show_size = show_size
            self.clear()
            # Last pts/dts values are never cleared
            self.last_dts = 0
            self.last_pts = 0

        def clear(self):
            self.count = 0
            self.duration = 0.0
            self.size = 0

        def to_str_dt(self, dt, level=0):
            tags = ['%.2f FPS' % (self.count / dt)]
            if level >= 2:
                tags.append('%.1f%% miss' % ((dt - self.duration) / dt))
            if level >= 1 and self._show_size:
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
    def _on_da_key_release(self, src, evt):
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
            self.stats_logger.warn('Frames dropped: %s', '; '.join(drop_tags))
        self.qos_dropped_info.clear()

        # Merge in information from jitterbuffer
        if self.last_jitterbuffer is not None:
            jstats = self.last_jitterbuffer.get_property('stats')

            # Units guessed
            self.extra_stats['rtt'] = '%.1fms' % (
                jstats.get_uint64('rtx-rtt')[1] / 1.e6)

            if self.DUMP_EXTRA_EVENTS:
                self.extra_stats['rtx-pp'] = '%.1f' % (
                    jstats.get_double('rtx-per-packet')[1])

            old_jstats = self.last_jitterbuffer_stats
            new_stats = dict(
                rtx_count=jstats.get_uint64('rtx-count')[1],
                rtx_success_count=jstats.get_uint64('rtx-success-count')[1])
            self.last_jitterbuffer_stats = new_stats
            diff_stats = { name: val - old_jstats.get(name, 0)
                           for name, val in new_stats.items() }
            if diff_stats['rtx_success_count']:
                # Successfully retransmitted
                self.extra_stats['rtx-ok'] = diff_stats['rtx_success_count']

            if diff_stats['rtx_count'] != diff_stats['rtx_success_count']:
                self.extra_stats['loss-rtx'] = (
                    diff_stats['rtx_count'] - diff_stats['rtx_success_count'])
        else:
            # Should not happen when video is playing normally
            self.extra_stats['no-jtb'] = 1

        decoded_frames = self.detector_stats['decoded'].count

        # print out statistics from detectors
        det_tags = list()
        for name, value in sorted(self.detector_stats.items()):
            det_tags.append(
                '%s: %s' % (name, value.to_str_dt(dt, level=1)))
            value.clear()
        for name, value in sorted(self.extra_stats.items()):
            det_tags.append('%s: %s' % (name, value))
        self.extra_stats.clear()

        self.stats_logger.info(
            '; '.join(det_tags) or 'No frames detected')

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

def video_window_init():
    #GObject.threads_init()
    Gst.init(None)

def video_window_main():
    g_quit_handlers.append(Gtk.main_quit)
    Gtk.main()

if __name__ == '__main__':
    # testing only
    host, port_str = sys.argv[0].split(':')

    video_window_init()
    asyncio_misc_init()
    vv = VideoWindow(host, int(port))
    video_window_main()
