#!/usr/bin/env python
import math

from vui_helpers import add_pair, MemoryLoggingHandler

class OnScreenDisplay(object):
    def __init__(self, calibration):
        self.calibration = calibration

    def render_svg(self, out, ui_state, control_dict, server_state, logs):
        """Render SVG for a given state. Should not access anything other
        than parameters, as this function may be called during replay.
        """
        print >>out, '<svg width="{image_size[0]}" height="{image_size[1]}">'\
            .format(**ui_state)

        rmode = ui_state['reticle_mode']
        if rmode == 1:
            # Shooting reticle
            reticle_center_rel = add_pair((0.5, 0.5),
                                          ui_state['reticle_offset'])
            reticle_center = (
                ui_state['image_size'][0] * reticle_center_rel[0],
                ui_state['image_size'][1] * reticle_center_rel[1])

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
'''.format(ui_state, reticle_center)
        elif rmode == 2:
            # Calibration reticle

            # Create a list of calibration lines, each line is a list of points
            # in degrees.
            lines_deg = []
            # Start with a cross in the middle. Add many points, as lines are
            # not straight
            lines_deg.append([(x, 0) for x in xrange(-30, 31, 5)])
            lines_deg.append([(0, y) for y in xrange(-15, 16, 5)])
            # Add a couple of squares
            for s in (5, 10, 15):
                lines_deg.append([(-s, s), (0, s), (s, s), (s, 0), (s, -s),
                                  (0, -s), (-s, -s), (-s, 0), (-s, s)])

            # Re-use reticle_offset to allow reticle movement
            offs_x, offs_y = ui_state['reticle_offset']

            print >>out, '<g stroke="rgb(255,128,0)">'
            # Convert degrees to pixels. It may be faster to do all in one go,
            # but I do not care about it for now.
            lines_pix = []
            for one_line in lines_deg:
                pix = self.calibration.from_world2d(
                    ((math.tan(math.radians(pt[0] + offs_x)),
                      math.tan(math.radians(pt[1] + offs_y)))
                     for pt in one_line),
                    image_size=ui_state['image_size'])
                for p1, p2 in zip(pix, pix[1:]):
                    print >>out, (
                        '<line x1="%.2f" x2="%.2f" y1="%.2f" y2="%.2f"/>'
                        % (p1[0], p2[0], p1[1], p2[1]))
            print >>out, '</g>'

        status_lines = list()
        if not ui_state['status_on']:
            status_lines.append('[OFF]')
        else:
            # Add turret position
            if control_dict.get('turret') is None:
                status_lines.append('Turret OFF')
            else:
                status_lines.append('Turret: ({:+5.1f}, {:+5.1f})'
                                    .format(*control_dict['turret']))

            # Add GPIO status
            tags = list()
            if control_dict['laser_on']:
                tags.append('LAS')
            if control_dict['agitator_on']:
                tags.append('AGT')
            if control_dict['green_led_on']:
                tags.append('GRN')
            if not tags:
                tags.append('(all off)')
            status_lines.append(','.join(tags))

            # Add servo status
            if server_state:
                s_voltage = server_state.get('servo_voltage', {})
                s_status = server_state.get('servo_status', {})
                s_temp = server_state.get('servo_temp', {})
            else:
                s_voltage = s_status = s_temp = {}
            got_any = False
            for servo in sorted(set(s_voltage.keys() + s_status.keys() +
                                    s_temp.keys()),
                                key=int):
                tags = []
                if s_status.get(servo):
                    tags.append(str(s_status[servo]))
                if s_voltage.get(servo):
                    tags.append('%.2fV' % s_voltage[servo])
                if s_temp.get(servo):
                    tags.append('%.1fC' % s_temp[servo])
                if not tags:
                    continue
                tags.append("%-2s" % servo)
                status_lines.append(' '.join(tags))
                got_any = True
            if not got_any:
                status_lines.append('No servo status available')

            # TODO mafanasyev: when long time has expired since last motion,
            # make this output more distinct
            if "last_motion_time" not in server_state:
                status_lines.append('Gait unready')
            elif not server_state["last_motion_time"]:
                status_lines.append('Gait ready')
            else:
                dt = (server_state["srv_time"] -
                      server_state["last_motion_time"])
                msg = ('NO-MOVE %.1f sec' % dt)
                if (dt > 30.0) and (int(dt * 2.0) % 2):
                    # Flash exclamation sign if spent too long without motion
                    msg = "!!!!! " + msg
                status_lines.append(msg)


        # Always show autofire, even when status is off
        if ui_state.get('autofire_mode'):
            status_lines.append('[RCLICK AUTOFIRE %r]'
                                % ui_state['autofire_mode'])


        # We output each text twice: first text outline in black, then text
        # itself in bright color. This ensures good visibility over both black
        # and green background.
        for tp in ['stroke="black" fill="black"', 'fill="COLOR"']:
            print >>out, '''
<text transform="translate(10 {0})" {1}
   font-family="Helvetica,sans-serif"
   font-size="{2}" text-anchor="left" dominant-baseline="text-before-edge">
'''.format(ui_state['image_size'][1] - 15,
           tp.replace('COLOR', 'lime'), ui_state['msg_font_size'])
            # Total number of lines is specified in MemoryLoggingHandler
            # constructor.
            for line_num, mtuple in enumerate(reversed(logs)):
                line = MemoryLoggingHandler.to_string(mtuple)
                print >>out, '<tspan x="0" y="%d"><![CDATA[%s]]></tspan>' % (
                    (-1 - line_num) * ui_state['msg_font_size'], line)
            print >>out, '</text>'

            print >>out, '''
<text transform="translate({0} 15)" {1}
   font-family="Courier,fixed" font-weight="bold"
   font-size="{2}" text-anchor="end" dominant-baseline="text-before-edge">
'''.format(ui_state['image_size'][0] - 10, tp.replace('COLOR', 'white'),
           ui_state['msg_font_size'])
            for line_num, line in enumerate(status_lines):
                print >>out, ('<tspan x="0" y="%d"><![CDATA[%s]]></tspan>'
                              % (line_num * ui_state['msg_font_size'], line))
            print >>out, '</text>'

        print >>out, '</svg>'
