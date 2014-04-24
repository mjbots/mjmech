# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import copy

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

import ripple_gait
import settings
import tf

from mtool_common import BoolContext

PHASE_STEP = 0.01

# TODO jpieper: It would be nice to be able to change:
#  The projection axis.
#  The scale.
#  The coordinate frame used (world, robot, body)
class GaitGeometryDisplay(object):
    FRAME_ROBOT, FRAME_WORLD, FRAME_BODY = range(3)
    PROJECTION_XY, PROJECTION_YZ, PROJECTION_XZ = range(3)

    def __init__(self, ui):
        self.graphics_scene = QtGui.QGraphicsScene()
        self.graphics_view = ui.gaitGeometryView
        self.graphics_view.setScene(self.graphics_scene)
        self.graphics_view.setTransform(QtGui.QTransform().scale(1, -1))

        self.config = None
        self.state = None

        self.frame = self.FRAME_ROBOT
        self.projection = self.PROJECTION_XY
        self.scale = 300.0

    def set_view(self, frame, projection, scale):
        self.frame = frame
        self.projection = projection
        self.scale = scale

        if self.config is not None and self.state is not None:
            self.set_state(self.state)

    def set_gait_config(self, config):
        self.config = config
        self.graphics_scene.clear()

        # Things to render:
        #  * Body position
        #  * Body CoG
        #  * Shoulder positions
        #  * Leg positions
        #  * Stability polygon
        #
        # Nice to have: Coordinate axes and scale labels.

        body_poly = QtGui.QPolygonF([
                QtCore.QPointF(-10.0, -10.0),
                QtCore.QPointF(-10.0, 10.0),
                QtCore.QPointF(10.0, 10.0),
                QtCore.QPointF(10.0, -10.0)])
        self.body = self.graphics_scene.addPolygon(
            body_poly, QtGui.QPen(QtCore.Qt.black),
            QtGui.QBrush(QtCore.Qt.red))
        self.body.setFlags(QtGui.QGraphicsItem.ItemIgnoresTransformations)

        self.shoulders = {}
        self.legs = {}
        shoulder_poly = QtGui.QPolygonF([
                QtCore.QPointF(-10, -10),
                QtCore.QPointF(0, 10),
                QtCore.QPointF(10, -10)])
        for leg_num, leg in config.mechanical.leg_config.iteritems():
            this_shoulder = self.graphics_scene.addPolygon(
                shoulder_poly, QtGui.QPen(QtCore.Qt.black),
                QtGui.QBrush(QtCore.Qt.blue))
            this_shoulder.setFlags(QtGui.QGraphicsItem.ItemIgnoresTransformations)
            self.shoulders[leg_num] = this_shoulder

            this_leg = self.graphics_scene.addEllipse(
                -10, -10, 20, 20,
                 QtGui.QPen(QtCore.Qt.black),
                 QtGui.QBrush(QtCore.Qt.green))
            this_leg.setFlags(QtGui.QGraphicsItem.ItemIgnoresTransformations)
            self.legs[leg_num] = this_leg

    def _project(self, point, frame):
        target_frame = None
        if self.frame == self.FRAME_ROBOT:
            target_frame = self.state.robot_frame
        elif self.frame == self.FRAME_WORLD:
            target_frame = self.state.world_frame
        elif self.frame == self.FRAME_BODY:
            target_frame = self.state.body_frame

        target_point = target_frame.map_from_frame(frame, point)

        if self.projection == self.PROJECTION_XY:
            return (target_point.x, target_point.y)
        elif self.projection == self.PROJECTION_YZ:
            return (target_point.y, target_point.z)
        elif self.projection == self.PROJECTION_XZ:
            return (target_point.x, target_point.z)

    def set_state(self, state):
        assert self.config is not None
        self.state = state

        self.body.setPos(*self._project(tf.Point3D(), state.body_frame))

        for leg_num, shoulder in self.shoulders.iteritems():
            leg_config = self.config.mechanical.leg_config[leg_num]

            body_point = tf.Point3D(
                    leg_config.mount_x_mm,
                    leg_config.mount_y_mm,
                    leg_config.mount_z_mm)
            shoulder.setPos(*self._project(body_point, state.body_frame))

        for leg_num, leg_item in self.legs.iteritems():
            leg = state.legs[leg_num]

            leg_item.setPos(*self._project(leg.point, leg.frame))
            if leg.mode == ripple_gait.STANCE:
                color = QtCore.Qt.green
            elif leg.mode == ripple_gait.SWING:
                color = QtCore.Qt.yellow

            leg_item.setBrush(QtGui.QBrush(color))

        self.graphics_view.fitInView(-self.scale, -self.scale,
                                      2 * self.scale, 2 * self.scale)

class GaitGraphDisplay(object):
    def __init__(self, ui):
        self.graphics_scene = QtGui.QGraphicsScene()
        self.graphics_view = ui.gaitGraphView

        self.graphics_view.setScene(self.graphics_scene)
        self.phase_line = None
        self.phase = 0.0

    def fit_in_view(self):
        self.graphics_view.fitInView(QtCore.QRectF(-0.1, 0, 1.1, 1))

    def set_phase(self, phase):
        self.phase = phase

        if self.phase_line:
            pos = self.phase_line.pos()
            self.phase_line.setPos(phase, pos.y())

    def set_gait_graph(self, graph):
        self.graphics_scene.clear()

        leg_numbers = sorted(graph.leg.keys())
        count = len(leg_numbers)

        if count == 0:
            return

        self.graphics_scene.addRect(0., 0., 1., 1.)

        self.phase_line = self.graphics_scene.addLine(0., 0., 0., 1.0)

        y_offset = 0.0
        y_size = 1.0 / count
        for leg_number in leg_numbers:
            label = self.graphics_scene.addSimpleText("%d" % leg_number)
            label.setPos(-0.08, y_offset + 0.1 * y_size)
            label.setFlag(QtGui.QGraphicsItem.ItemIgnoresTransformations)

            self.graphics_scene.addLine(0, y_offset, 1.0, y_offset)

            old_phase = 0.0
            old_swing = False
            for phase, mode in graph.leg[leg_number].sequence:
                if mode == ripple_gait.STANCE and old_swing:
                    # Render a black bar for this swing phase.
                    self.graphics_scene.addRect(
                        old_phase, y_offset + 0.1 * y_size,
                        phase - old_phase, 0.8 * y_size,
                        QtGui.QPen(),
                        QtGui.QBrush(QtCore.Qt.black))

                old_phase = phase
                old_swing = True if mode == ripple_gait.SWING else False

            y_offset += y_size

        self.fit_in_view()

class GaitTab(object):
    def __init__(self, ui, ikconfig_tab, servo_tab):
        self.ui = ui
        self.ikconfig_tab = ikconfig_tab
        self.servo_tab = servo_tab

        self.in_number_changed = BoolContext()
        self.in_command_changed = BoolContext()

        self.ripple_config = ripple_gait.RippleConfig()
        self.ripple_gait = ripple_gait.RippleGait(self.ripple_config)

        self.command = ripple_gait.Command()

        self.current_states = []
        self.gait_graph_display = GaitGraphDisplay(self.ui)
        self.gait_geometry_display = GaitGeometryDisplay(self.ui)

        self.ui.gaitLegList.currentItemChanged.connect(self.handle_leg_change)

        for spin in [self.ui.mountingLegXSpin,
                     self.ui.mountingLegYSpin,
                     self.ui.mountingLegZSpin]:
            spin.valueChanged.connect(self.handle_leg_data_change)

        for spin in [self.ui.bodyCogXSpin,
                     self.ui.bodyCogYSpin,
                     self.ui.bodyCogZSpin,
                     self.ui.idlePositionXSpin,
                     self.ui.idlePositionYSpin,
                     self.ui.idlePositionZSpin,
                     self.ui.maxCycleTimeSpin,
                     self.ui.liftHeightSpin,
                     self.ui.minSwingPercentSpin,
                     self.ui.maxSwingPercentSpin,
                     self.ui.bodyZOffsetSpin]:
            spin.valueChanged.connect(self.handle_gait_config_change)

        self.command_spins = [self.ui.commandXSpin,
                              self.ui.commandYSpin,
                              self.ui.commandRotSpin,
                              self.ui.commandBodyXSpin,
                              self.ui.commandBodyYSpin,
                              self.ui.commandBodyZSpin,
                              self.ui.commandPitchSpin,
                              self.ui.commandRollSpin,
                              self.ui.commandYawSpin]

        for spin in self.command_spins:
            spin.valueChanged.connect(self.handle_command_change)

        self.ui.commandResetButton.clicked.connect(self.handle_command_reset)

        for combo in [self.ui.geometryFrameCombo,
                      self.ui.geometryProjectionCombo]:
            combo.currentIndexChanged.connect(self.handle_geometry_change)
        self.ui.geometryScaleSpin.valueChanged.connect(
            self.handle_geometry_change)

        self.ui.tabWidget.currentChanged.connect(self.handle_current_changed)

        self.ui.playbackBeginCombo.currentIndexChanged.connect(
            self.handle_playback_config_change)
        self.ui.playbackPhaseSlider.setMaximum(int(1.0 / PHASE_STEP))
        self.ui.playbackPhaseSlider.valueChanged.connect(
            self.handle_playback_phase_change)

    def get_float_configs(self):
        return [(self.ui.bodyCogXSpin, 'body_cog_x_mm'),
                (self.ui.bodyCogYSpin, 'body_cog_y_mm'),
                (self.ui.bodyCogZSpin, 'body_cog_z_mm'),
                (self.ui.idlePositionXSpin, 'idle_position_x_mm'),
                (self.ui.idlePositionYSpin, 'idle_position_y_mm'),
                (self.ui.idlePositionZSpin, 'idle_position_z_mm'),
                (self.ui.maxCycleTimeSpin, 'max_cycle_time_s'),
                (self.ui.liftHeightSpin, 'lift_height_mm'),
                (self.ui.minSwingPercentSpin, 'min_swing_percent'),
                (self.ui.maxSwingPercentSpin, 'max_swing_percent'),
                (self.ui.bodyZOffsetSpin, 'body_z_offset_mm'),
                (self.ui.geometryScaleSpin, 'geometry_scale_mm'),
                (self.ui.commandXSpin, 'command_x_mm_s'),
                (self.ui.commandYSpin, 'command_y_mm_s'),
                (self.ui.commandRotSpin, 'command_rot_deg_s'),
                (self.ui.commandBodyXSpin, 'command_body_x_mm'),
                (self.ui.commandBodyYSpin, 'command_body_y_mm'),
                (self.ui.commandBodyZSpin, 'command_body_z_mm'),
                (self.ui.commandPitchSpin, 'command_body_pitch_deg'),
                (self.ui.commandRollSpin, 'command_body_roll_deg'),
                (self.ui.commandYawSpin, 'command_body_yaw_deg'),
                ]

    def string_to_leg_config(self, value):
        assert isinstance(value, str)
        fields = [float(x) for x in value.split(',')]
        assert len(fields) >= 3

        result = ripple_gait.LegConfig()
        result.mount_x_mm = fields[0]
        result.mount_y_mm = fields[1]
        result.mount_z_mm = fields[2]

        return result

    def leg_config_to_string(self, leg_config):
        assert isinstance(leg_config, ripple_gait.LegConfig)
        return '%.2f,%.2f,%.2f' % (
            leg_config.mount_x_mm,
            leg_config.mount_y_mm,
            leg_config.mount_z_mm)

    def read_settings(self, config):
        if not config.has_section('gaitconfig'):
            return

        for spin, name in self.get_float_configs():
            if config.has_option('gaitconfig', name):
                spin.setValue(config.getfloat('gaitconfig', name))

        def set_combo(combo, name):
            settings.restore_combo(config, 'gaitconfig', combo, name)

        set_combo(self.ui.geometryFrameCombo, 'geometry_frame')
        set_combo(self.ui.geometryProjectionCombo, 'geometry_projection')

        if config.has_section('gaitconfig.legs'):
            for leg_name, value in config.items('gaitconfig.legs'):
                leg_num = int(leg_name.split('.')[1])
                self.ripple_config.mechanical.leg_config[leg_num] = \
                    self.string_to_leg_config(value)

        self.handle_leg_change(self.ui.gaitLegList.currentItem())
        self.handle_gait_config_change()
        self.handle_geometry_change()

    def write_settings(self, config):
        config.add_section('gaitconfig')
        for spin, name in self.get_float_configs():
            config.set('gaitconfig', name, spin.value())

        config.set('gaitconfig', 'geometry_frame',
                   self.ui.geometryFrameCombo.currentText())
        config.set('gaitconfig', 'geometry_projection',
                   self.ui.geometryProjectionCombo.currentText())

        config.add_section('gaitconfig.legs')
        for leg_data in self.ripple_config.mechanical.leg_config.iteritems():
            leg_num, leg_config = leg_data
            config.set('gaitconfig.legs', 'leg.%d' % leg_num,
                       self.leg_config_to_string(leg_config))

    def handle_current_changed(self, index=2):
        if index != 2:
            return

        # Update the leg list widget.
        available_legs = self.ikconfig_tab.get_all_legs()
        enabled_legs = set(self.ikconfig_tab.get_enabled_legs())

        for leg_num in available_legs:
            leg_str = str(leg_num)
            if not self.ui.gaitLegList.findItems(
                  leg_str, QtCore.Qt.MatchExactly):
                self.ui.gaitLegList.addItem(leg_str)
            items = self.ui.gaitLegList.findItems(
                leg_str, QtCore.Qt.MatchExactly)
            item = items[0]
            if leg_num in enabled_legs:
                item.setFlags(QtCore.Qt.ItemIsEnabled |
                              QtCore.Qt.ItemIsSelectable)
            else:
                item.setFlags(0)

        self.handle_leg_change(self.ui.gaitLegList.currentItem())

        # Make sure that our configuration is fully up to date.
        self.handle_gait_config_change()

    def handle_leg_change(self, current_item):
        widgets = [self.ui.mountingLegXSpin,
                   self.ui.mountingLegYSpin,
                   self.ui.mountingLegZSpin]

        if current_item is None:
            [x.setEnabled(False) for x in widgets]
            return

        [x.setEnabled(True) for x in widgets]

        with self.in_number_changed:
            number = int(current_item.text())

            leg_config = self.ripple_config.mechanical.leg_config.get(
                number, ripple_gait.LegConfig())
            if leg_config.mount_x_mm is not None:
                self.ui.mountingLegXSpin.setValue(leg_config.mount_x_mm)
            if leg_config.mount_y_mm is not None:
                self.ui.mountingLegYSpin.setValue(leg_config.mount_y_mm)
            if leg_config.mount_z_mm is not None:
                self.ui.mountingLegZSpin.setValue(leg_config.mount_z_mm)

        self.handle_leg_data_change()

    def handle_leg_data_change(self):
        if self.in_number_changed.value:
            return

        number_item = self.ui.gaitLegList.currentItem()
        if number_item is None:
            return

        number = int(number_item.text())

        leg_config = self.ripple_config.mechanical.leg_config.get(
            number, ripple_gait.LegConfig())

        leg_config.mount_x_mm = self.ui.mountingLegXSpin.value()
        leg_config.mount_y_mm = self.ui.mountingLegYSpin.value()
        leg_config.mount_z_mm = self.ui.mountingLegZSpin.value()

        self.ripple_config.mechanical.leg_config[number] = leg_config

        self.handle_gait_config_change()

    def handle_gait_config_change(self):
        # Put all of our GUI information into the RippleGait that
        # needs to be.

        mechanical = self.ripple_config.mechanical

        for item_num in range(self.ui.gaitLegList.count()):
            item = self.ui.gaitLegList.item(item_num)
            leg_number = int(item.text())
            if not (item.flags() & QtCore.Qt.ItemIsEnabled):
                if leg_number in mechanical.leg_config:
                    del mechanical.leg_config[leg_number]
            elif leg_number not in mechanical.leg_config:
                mechanical.leg_config[leg_number] = ripple_gait.LegConfig()

        for leg_data in self.ripple_config.mechanical.leg_config.iteritems():
            leg_number, leg_config = leg_data

            leg_config.idle_x_mm = self.ui.idlePositionXSpin.value()
            leg_config.idle_y_mm = self.ui.idlePositionYSpin.value()
            leg_config.idle_z_mm = self.ui.idlePositionZSpin.value()

            leg_config.leg_ik = self.ikconfig_tab.get_leg_ik(leg_number)

        self.ripple_config.mechanical.body_cog_x_mm = \
            self.ui.bodyCogXSpin.value()
        self.ripple_config.mechanical.body_cog_y_mm = \
            self.ui.bodyCogYSpin.value()
        self.ripple_config.mechanical.body_cog_z_mm = \
            self.ui.bodyCogZSpin.value()

        self.ripple_config.max_cycle_time_s = self.ui.maxCycleTimeSpin.value()
        self.ripple_config.lift_height_mm = self.ui.liftHeightSpin.value()
        self.ripple_config.min_swing_percent = \
            self.ui.minSwingPercentSpin.value()
        self.ripple_config.max_swing_percent = \
            self.ui.maxSwingPercentSpin.value()

        self.ripple_config.leg_order = \
            self.ripple_config.mechanical.leg_config.keys()

        self.ripple_config.body_z_offset = self.ui.bodyZOffsetSpin.value()

        self.ripple_gait = ripple_gait.RippleGait(self.ripple_config)

        self.gait_geometry_display.set_gait_config(self.ripple_config)

        self.update_gait_graph()
        self.handle_playback_config_change()

    def update_gait_graph(self):
        self.gait_graph_display.set_gait_graph(self.ripple_gait.get_gait_graph())

    def handle_playback_config_change(self):
        # Re-run the playback recording the state through an entire
        # phase.  Then make sure that the graphic state is current for
        # the phase that is selected now.

        begin_index = self.ui.playbackBeginCombo.currentIndex()

        if begin_index == 0: # Idle
            begin_state = self.ripple_gait.get_idle_state()
        else:
            assert False, 'Unsupported begin state'

        begin_state = self.ripple_gait.set_state(begin_state, self.command)

        self.current_states = (
            [copy.deepcopy(begin_state)] +
            [copy.deepcopy(self.ripple_gait.advance_phase(PHASE_STEP))
             for x in range(int(1.0 / PHASE_STEP))])

        self.handle_playback_phase_change()

    def handle_playback_phase_change(self):
        self.update_phase(self.ui.playbackPhaseSlider.value() * PHASE_STEP)

    def update_phase(self, phase):
        # Render the phase line in the gait graph.
        self.gait_graph_display.set_phase(phase)

        # Update the current geometry rendering.
        state = self.current_states[int(phase / PHASE_STEP)]

        self.gait_geometry_display.set_state(state)

    def handle_geometry_change(self):
        frame = [GaitGeometryDisplay.FRAME_ROBOT,
                 GaitGeometryDisplay.FRAME_WORLD,
                 GaitGeometryDisplay.FRAME_BODY][
            self.ui.geometryFrameCombo.currentIndex()]

        projection = [GaitGeometryDisplay.PROJECTION_XY,
                      GaitGeometryDisplay.PROJECTION_YZ,
                      GaitGeometryDisplay.PROJECTION_XZ][
            self.ui.geometryProjectionCombo.currentIndex()]

        self.gait_geometry_display.set_view(
            frame, projection, self.ui.geometryScaleSpin.value())

    def handle_command_change(self):
        if self.in_command_changed.value:
            return

        with self.in_command_changed:
            self.command.translate_x_mm_s = self.ui.commandXSpin.value()
            self.command.translate_y_mm_s = self.ui.commandYSpin.value()
            self.command.rotate_deg_s = self.ui.commandRotSpin.value()
            self.command.body_x_mm = self.ui.commandBodyXSpin.value()
            self.command.body_y_mm = self.ui.commandBodyYSpin.value()
            self.command.body_z_mm = self.ui.commandBodyZSpin.value()
            self.command.body_pitch_deg = self.ui.commandPitchSpin.value()
            self.command.body_roll_deg = self.ui.commandRollSpin.value()
            self.command.body_yaw_deg = self.ui.commandYawSpin.value()

            self.handle_playback_config_change()

    def handle_command_reset(self):
        with self.in_command_changed:
            for spin in self.command_spins:
                spin.setValue(0.0)

        self.handle_command_change()
