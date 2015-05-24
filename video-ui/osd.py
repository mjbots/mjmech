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

        # Create a list of calibration lines, each entry may be either:
        #  - string -- this defines properties for subsequent lines
        #  - list points, each is a pair of angles in degrees
        lines_deg = []

        rmode = ui_state['reticle_mode']
        if rmode == 1:
            # Shooting reticle

            co = 15    # Size of thick outer cross
            ci = 5     # Size of thin inner cross
            pls = 1.0  # Parallel line step
            lines_deg += [
                # Thick outer cross
                'stroke-width="4"',
                [(-co, 0), (-ci, 0)], [(ci, 0), (co, 0)],
                [(0, -co), (0, -ci)], [(0, ci), (0, co)],
                # Smaller inner cross
                '',
                [(-ci, 0), (ci, 0)], [(0, -ci), (0, ci)],
                # Top ranging line
                [(-0.5 * ci, -pls), (0.5 * ci, -pls)],
                ]
            for num in xrange(1, 6):
                width = ci * (10 - num) / 10.0
                lines_deg.append([(-width, pls * num), (width, pls * num)])

        elif rmode == 2:
            # Calibration reticle

            # Start with a cross in the middle. Add many points, as lines are
            # not straight
            lines_deg.append([(x, 0) for x in xrange(-30, 31, 5)])
            lines_deg.append([(0, y) for y in xrange(-15, 16, 5)])
            # Add a couple of squares
            for s in (5, 10, 15):
                lines_deg.append([(-s, s), (0, s), (s, s), (s, 0), (s, -s),
                                  (0, -s), (-s, -s), (-s, 0), (-s, s)])


        turret_actual = server_state.get("turret_position", (None, None))
        if rmode and (turret_actual[0] is not None) and (
            turret_actual[1] is not None) and control_dict.get('turret'):
            # Draw 'actual position' mark
            actual_x = control_dict['turret'][0] - turret_actual[0]
            actual_y = control_dict['turret'][1] - turret_actual[1]
            lines_deg.append("stroke='yellow'")

            # Show 45 deg square with diagonal of 3 counts
            sz = 0.325 * 3 / 2
            lines_deg.append([(actual_x - sz, actual_y),
                              (actual_x, actual_y + sz),
                              (actual_x + sz, actual_y),
                              (actual_x, actual_y - sz),
                              (actual_x - sz, actual_y)])

            # When we are moving, draw a bigger cross around it
            if server_state.get('turret_inmotion'):
                sz2 = 10.0
                lines_deg.append([(actual_x - sz2, actual_y),
                                  (actual_x + sz2, actual_y)])
                lines_deg.append([(actual_x, actual_y - sz2),
                                  (actual_x, actual_y + sz2)])

        if lines_deg:
            # Re-use reticle_offset to allow reticle movement
            offs_x, offs_y = ui_state['reticle_offset']

            print >>out, '<g stroke="rgb(255,128,0)">'
            # Convert degrees to pixels. It may be faster to do all in one go,
            # but I do not care about it for now.
            lines_pix = []
            line_flags = ""
            for one_line in lines_deg:
                if isinstance(one_line, str):
                    line_flags = one_line
                    continue
                pix = self.calibration.from_world2d(
                    ((math.tan(math.radians(pt[0] + offs_x)),
                      math.tan(math.radians(pt[1] + offs_y)))
                     for pt in one_line),
                    image_size=ui_state['image_size'])
                for p1, p2 in zip(pix, pix[1:]):
                    print >>out, (
                        '<line x1="%.2f" x2="%.2f" y1="%.2f" y2="%.2f" %s/>'
                        % (p1[0], p2[0], p1[1], p2[1], line_flags))
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

            # Add Z position
            if (control_dict.get('gait') and
                control_dict['gait'].get('body_z_mm') is not None):
                status_lines.append(
                    'Z: %d' % control_dict['gait']['body_z_mm'])

            status_lines.append('speed: %d' % ui_state['speed'])

            # Add GPIO status
            tags = list()
            if control_dict['laser_on']:
                tags.append('LAS')
            if control_dict['green_led_on']:
                tags.append('GRN')
            if control_dict['agitator_mode'] == 2:
                tags.append('AGT-FORCE-ON')
            elif control_dict['agitator_mode'] == 0:
                tags.append('AGT-FORCE-OFF')
            elif server_state.get('agitator_on'):
                tags.append('AGT-ON')  # Auto mode, on
            else:
                tags.append('AGT-AUTO')  # Auto mode

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

            if server_state.get("shots_fired"):
                status_lines.append("Shots fired: %d" %
                                    server_state["shots_fired"])


        # Always show autofire, even when status is off
        if ui_state.get('autofire_mode'):
            status_lines.append('[RCLICK AUTOFIRE %r]'
                                % ui_state['autofire_mode'])


        # We output each text twice: first text outline in black, then text
        # itself in bright color. This ensures good visibility over both black
        # and green background.
        for tp in [('stroke="black" fill="black" '+
                    'stroke-width="5" stroke-linecap="round"'),
                   'fill="COLOR"']:
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
                    (-1 - line_num) * (ui_state['msg_font_size'] + 2), line)
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
