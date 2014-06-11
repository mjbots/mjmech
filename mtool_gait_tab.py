# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import functools
import numpy
import time
import sys

import trollius as asyncio
from trollius import From, Task

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

import convexhull
import ripple_gait
import servo_controller
import settings
import tf

from mtool_common import BoolContext
import mtool_graphics_scene

PLAYBACK_TIMEOUT_MS = 40

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

        self.axes_item = mtool_graphics_scene.AxesItem(
            true_scale=True, grid_skip=2)
        self.axes_item.x_scale = self.scale
        self.axes_item.y_scale = self.scale
        self.axes_item.x_suffix = 'mm'
        self.axes_item.y_suffix = 'mm'
        self.graphics_scene.addItem(self.axes_item)

        self.support_poly = QtGui.QGraphicsPolygonItem()
        self.support_poly.setPen(QtGui.QColor(0, 0, 0, 0))
        self.support_poly.setBrush(QtGui.QBrush(QtGui.QColor(0, 128, 0, 15)))
        self.graphics_scene.addItem(self.support_poly)

        self.items = []

    def resize(self):
        self.graphics_view.fitInView(-self.scale, -self.scale,
                                      2 * self.scale, 2 * self.scale,
                                      QtCore.Qt.KeepAspectRatio)

    def set_view(self, frame, projection, scale):
        self.frame = frame
        self.projection = projection
        self.scale = scale
        self.axes_item.x_scale = scale
        self.axes_item.y_scale = scale

        if self.config is not None and self.state is not None:
            self.set_state(self.state)

        self.axes_item.update()

    def set_gait_config(self, config):
        self.config = config

        for item in self.items:
            self.graphics_scene.removeItem(item)

        self.items = []

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
        self.items.append(self.body)

        self.cog = self.graphics_scene.addEllipse(
            -5, -5, 10, 10,
             QtGui.QPen(QtCore.Qt.black),
             QtGui.QBrush(QtCore.Qt.yellow))
        self.cog.setFlags(QtGui.QGraphicsItem.ItemIgnoresTransformations)
        self.items.append(self.cog)

        self.shoulders = {}
        self.legs = {}
        shoulder_poly = QtGui.QPolygonF([
                QtCore.QPointF(-10, 0),
                QtCore.QPointF(0, 10),
                QtCore.QPointF(10, 0),
                QtCore.QPointF(0, -10)])
        for leg_num, leg in config.mechanical.leg_config.iteritems():
            this_shoulder = self.graphics_scene.addPolygon(
                shoulder_poly, QtGui.QPen(QtCore.Qt.black),
                QtGui.QBrush(QtCore.Qt.blue))
            this_shoulder.setFlags(
                QtGui.QGraphicsItem.ItemIgnoresTransformations)
            self.shoulders[leg_num] = this_shoulder
            self.items.append(this_shoulder)

            this_leg = self.graphics_scene.addEllipse(
                -10, -10, 20, 20,
                 QtGui.QPen(QtCore.Qt.black),
                 QtGui.QBrush(QtCore.Qt.green))
            this_leg.setFlags(QtGui.QGraphicsItem.ItemIgnoresTransformations)
            self.legs[leg_num] = this_leg
            self.items.append(this_leg)

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

        stance_points = []

        self.body.setPos(*self._project(tf.Point3D(), state.body_frame))
        self.cog.setPos(*self._project(tf.Point3D(), state.cog_frame))

        for leg_num, shoulder in self.shoulders.iteritems():
            if leg_num not in state.legs:
                continue
            shoulder.setPos(*self._project(tf.Point3D(),
                                           state.legs[leg_num].shoulder_frame))

        for leg_num, leg_item in self.legs.iteritems():
            if leg_num not in state.legs:
                continue
            leg = state.legs[leg_num]

            point = self._project(leg.point, leg.frame)
            leg_item.setPos(*point)

            shoulder_point = leg.shoulder_frame.map_from_frame(
                leg.frame, leg.point)
            ik_result = leg.leg_ik.do_ik(shoulder_point)

            if ik_result is None:
                color = QtCore.Qt.red
            elif leg.mode == ripple_gait.STANCE:
                color = QtCore.Qt.green
                stance_points.append(point)
            elif leg.mode == ripple_gait.SWING:
                color = QtCore.Qt.yellow
            else:
                assert False, 'unknown leg mode %d' % leg.mode

            leg_item.setBrush(QtGui.QBrush(color))

        if len(stance_points) >= 3:
            self.support_poly.setVisible(True)
            hull = convexhull.convexHull(stance_points)

            poly = QtGui.QPolygonF([QtCore.QPointF(x, y) for x, y in hull])
            self.support_poly.setPolygon(poly)
        else:
            self.support_poly.setVisible(False)

        self.resize()

class GaitGraphDisplay(object):
    def __init__(self, ui):
        self.graphics_scene = QtGui.QGraphicsScene()
        self.graphics_view = ui.gaitGraphView

        self.graphics_view.setScene(self.graphics_scene)
        self.phase_line = None
        self.phase = 0.0

    def resize(self):
        self.fit_in_view()

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

class CommandWidget(object):
    ATTR_NAMES = ['translate_x_mm_s',
                  'translate_y_mm_s',
                  'rotate_deg_s',
                  'body_x_mm',
                  'body_y_mm',
                  'body_z_mm',
                  'body_pitch_deg',
                  'body_roll_deg',
                  'body_yaw_deg',]

    ATTR_SUFFIXES = ['mm/s',
                     'mm/s',
                     'deg/s',
                     'mm',
                     'mm',
                     'mm',
                     'deg',
                     'deg',
                     'deg',]

    def __init__(self, ui, command, command_change_callback):
        self.ui = ui
        self.parent_command = command
        self.command_change_callback = command_change_callback
        self.scales = [ 400.0, 400.0, 50.0,
                        100.0, 100.0, 100.0,
                        45.0, 45.0, 45.0 ]

        self.update_lock = asyncio.Lock()

        self.config = None
        self.command = None

        self.graphics_scene = mtool_graphics_scene.GraphicsScene()
        self.graphics_scene.sceneMouseMoveEvent.connect(
            self.handle_mouse_move)
        self.graphics_scene.sceneMousePressEvent.connect(
            self.handle_mouse_press)
        self.graphics_view = self.ui.gaitCommandView
        self.graphics_view.setTransform(QtGui.QTransform().scale(1, -1))
        self.graphics_view.setScene(self.graphics_scene)

        self.in_scale_changed = BoolContext()

        for combo in [self.ui.commandXCombo,
                      self.ui.commandYCombo]:
            combo.currentIndexChanged.connect(self.handle_axis_change)

        for spin in [self.ui.commandXScaleSpin,
                     self.ui.commandYScaleSpin]:
            spin.valueChanged.connect(self.handle_scale_change)

        self.axes_item = mtool_graphics_scene.AxesItem()
        self.graphics_scene.addItem(self.axes_item)

        self.grid_count = 10
        self.usable_rects = {}
        for x in range(-self.grid_count + 1, self.grid_count):
            for y in range(-self.grid_count + 1, self.grid_count):
                self.usable_rects[(x, y)] = \
                    self.graphics_scene.addRect(
                    (x - 0.5) / self.grid_count,
                    (y - 0.5) / self.grid_count,
                    1.0 / self.grid_count, 1.0 / self.grid_count)

        for rect in self.usable_rects.itervalues():
            rect.setPen(QtGui.QPen(QtCore.Qt.NoPen))
            rect.setZValue(-20)

    def resize(self):
        self.fit_in_view()

    def fit_in_view(self):
        self.graphics_view.fitInView(QtCore.QRectF(-1, -1, 2, 2))

    def read_settings(self, config):
        def set_combo(combo, name):
            settings.restore_combo(config, 'gaitconfig', combo, name)

        set_combo(self.ui.commandXCombo, 'command_x_axis')
        set_combo(self.ui.commandYCombo, 'command_y_axis')

        for index in range(len(self.scales)):
            name = 'command_axis_scale_%d' % index
            if config.has_option('gaitconfig', name):
                self.scales[index] = config.getfloat('gaitconfig', name)

        self.handle_axis_change()

    def write_settings(self, config):
        config.set('gaitconfig', 'command_x_axis',
                   self.ui.commandXCombo.currentText())
        config.set('gaitconfig', 'command_y_axis',
                   self.ui.commandYCombo.currentText())

        for index, value in enumerate(self.scales):
            config.set('gaitconfig', 'command_axis_scale_%d' % index, value)

    def handle_axis_change(self):
        with self.in_scale_changed:
            self.ui.commandXScaleSpin.setValue(self.x_scale())
            self.ui.commandYScaleSpin.setValue(self.y_scale())

            self.axes_item.x_scale = self.x_scale()
            self.axes_item.y_scale = self.y_scale()
            self.axes_item.x_suffix = self.ATTR_SUFFIXES[self.x_axis()]
            self.axes_item.y_suffix = self.ATTR_SUFFIXES[self.y_axis()]

            self.axes_item.update()

            if self.config is not None:
                self.update_allowable(self.config, self.command)

    def handle_scale_change(self, value):
        if self.in_scale_changed.value:
            return

        with self.in_scale_changed:
            if self.x_axis() == self.y_axis():
                self.ui.commandXScaleSpin.setValue(value)
                self.ui.commandYScaleSpin.setValue(value)

            self.scales[self.x_axis()] = self.ui.commandXScaleSpin.value()
            self.scales[self.y_axis()] = self.ui.commandYScaleSpin.value()

            self.axes_item.x_scale = self.x_scale()
            self.axes_item.y_scale = self.y_scale()
            self.axes_item.update()

            if self.config is not None:
                self.update_allowable(self.config, self.command)

    def x_axis(self):
        return self.ui.commandXCombo.currentIndex()

    def y_axis(self):
        return self.ui.commandYCombo.currentIndex()

    def x_scale(self):
        return self.scales[self.x_axis()]

    def y_scale(self):
        return self.scales[self.y_axis()]

    def handle_mouse_move(self, cursor):
        x_value = cursor.x() * self.x_scale()
        y_value = cursor.y() * self.y_scale()

        if self.x_axis() == self.y_axis():
            x_value = y_value = 0.5 * (x_value + y_value)

        setattr(self.parent_command,
                self.ATTR_NAMES[self.x_axis()], x_value)
        setattr(self.parent_command,
                self.ATTR_NAMES[self.y_axis()], y_value)

        self.command_change_callback()

    def handle_mouse_press(self, cursor):
        self.handle_mouse_move(cursor)

    def update_allowable(self, config, command):
        self.next_config = config.copy()
        self.next_command = command.copy()

        for (x, y), rect in self.usable_rects.iteritems():
            old_brush = rect.brush()
            old_color = old_brush.color()
            rect.setBrush(QtGui.QBrush(QtGui.QColor(
                        old_color.red(),
                        old_color.green(),
                        old_color.blue(),
                        64)))

        Task(self._really_update_allowable())

    @asyncio.coroutine
    def _really_update_allowable(self):
        if self.update_lock.locked():
            return

        yield From(self.update_lock.acquire())

        try:
            while self.next_config is not None:
                yield From(self.do_update_allowable())
        finally:
            self.update_lock.release()

    @asyncio.coroutine
    def do_update_allowable(self):
        self.config = self.next_config
        self.command = self.next_command

        my_gait = ripple_gait.RippleGait(self.next_config)
        my_command = self.next_command.copy()

        self.next_config = None
        self.next_command = None

        next_wait = time.time() + 0.5 * 0.001 * PLAYBACK_TIMEOUT_MS

        for (x, y), rect in self.usable_rects.iteritems():
            x_value = self.x_scale() * float(x) / self.grid_count
            y_value = self.y_scale() * float(y) / self.grid_count

            setattr(my_command, self.ATTR_NAMES[self.x_axis()], x_value)
            setattr(my_command, self.ATTR_NAMES[self.y_axis()], y_value)

            color = (0, 255, 0)
            try:
                my_gait.set_command(my_command)
                actual_command = my_gait.command
                actual_x_value = getattr(
                    actual_command, self.ATTR_NAMES[self.x_axis()])
                actual_y_value = getattr(
                    actual_command, self.ATTR_NAMES[self.y_axis()])
                if actual_x_value != x_value or actual_y_value != y_value:
                    color = (255, 255, 0)
            except ripple_gait.NotSupported:
                color = (255, 0, 0)

            rect.setBrush(QtGui.QBrush(QtGui.QColor(*color)))

            if time.time() > next_wait:
                yield From(asyncio.sleep(0.5 * 0.001 * PLAYBACK_TIMEOUT_MS))
                if self.next_config is not None:
                    return
                next_wait = time.time() + 0.5 * 0.001 * PLAYBACK_TIMEOUT_MS

class GaitTab(object):
    (PLAYBACK_IDLE,
     PLAYBACK_SINGLE,
     PLAYBACK_REPEAT,
     PLAYBACK_SLOW_REPEAT) = range(4)

    def __init__(self, ui, ikconfig_tab, servo_tab):
        self.ui = ui
        self.ikconfig_tab = ikconfig_tab
        self.servo_tab = servo_tab

        self.current_command = None
        self.skipped_update = 0

        self.playback_mode = self.PLAYBACK_IDLE

        self.in_number_changed = BoolContext()
        self.in_command_changed = BoolContext()

        self.ripple_config = ripple_gait.RippleConfig()
        self.ripple_gait = ripple_gait.RippleGait(self.ripple_config)

        self.command = ripple_gait.Command()

        self.command_widget = CommandWidget(
            ui, self.command, self.handle_widget_set_command)

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
                     self.ui.swingPercentSpin,
                     self.ui.positionMarginSpin,
                     self.ui.bodyZOffsetSpin,
                     self.ui.staticCenterSpin,
                     self.ui.staticStableSpin,
                     self.ui.staticMarginSpin]:
            spin.valueChanged.connect(self.handle_gait_config_change)

        self.ui.staticEnableCheck.toggled.connect(
            self.handle_gait_config_change)

        self.ui.legOrderEdit.editingFinished.connect(
            self.handle_leg_order_editing_finished)

        self.command_spins = [
            (self.ui.commandXSpin, 'translate_x_mm_s'),
            (self.ui.commandYSpin, 'translate_y_mm_s'),
            (self.ui.commandRotSpin, 'rotate_deg_s'),
            (self.ui.commandBodyXSpin, 'body_x_mm'),
            (self.ui.commandBodyYSpin, 'body_y_mm'),
            (self.ui.commandBodyZSpin, 'body_z_mm'),
            (self.ui.commandPitchSpin, 'body_pitch_deg'),
            (self.ui.commandRollSpin, 'body_roll_deg'),
            (self.ui.commandYawSpin, 'body_yaw_deg'),
            ]

        for spin, _ in self.command_spins:
            spin.valueChanged.connect(self.handle_command_change)

        self.ui.commandResetButton.clicked.connect(self.handle_command_reset)

        for combo in [self.ui.geometryFrameCombo,
                      self.ui.geometryProjectionCombo]:
            combo.currentIndexChanged.connect(self.handle_geometry_change)
        self.ui.geometryScaleSpin.valueChanged.connect(
            self.handle_geometry_change)

        self.ui.tabWidget.currentChanged.connect(self.handle_current_changed)

        self.phase_step = 2.0 / self.ui.playbackPhaseSlider.maximum()

        self.ui.playbackBeginCombo.currentIndexChanged.connect(
            self.handle_playback_config_change)
        self.ui.playbackPhaseSlider.valueChanged.connect(
            self.handle_playback_phase_change)

        for button, state in [
            (self.ui.playbackSingleButton, self.PLAYBACK_SINGLE),
            (self.ui.playbackRepeatButton, self.PLAYBACK_REPEAT),
            (self.ui.playbackSlowRepeatButton, self.PLAYBACK_SLOW_REPEAT)]:

            button.toggled.connect(
                functools.partial(self.handle_playback_state_change, state))

        self.playback_timer = QtCore.QTimer()
        self.playback_timer.timeout.connect(self.handle_playback_timer)

    def resizeEvent(self, event):
        if self.ui.tabWidget.currentIndex() == 2:
            self.gait_graph_display.resize()
            self.gait_geometry_display.resize()
            self.command_widget.resize()

    def get_float_configs(self):
        return [(self.ui.bodyCogXSpin, 'body_cog_x_mm'),
                (self.ui.bodyCogYSpin, 'body_cog_y_mm'),
                (self.ui.bodyCogZSpin, 'body_cog_z_mm'),
                (self.ui.idlePositionXSpin, 'idle_position_x_mm'),
                (self.ui.idlePositionYSpin, 'idle_position_y_mm'),
                (self.ui.idlePositionZSpin, 'idle_position_z_mm'),
                (self.ui.maxCycleTimeSpin, 'max_cycle_time_s'),
                (self.ui.liftHeightSpin, 'lift_height_mm'),
                (self.ui.swingPercentSpin, 'swing_percent'),
                (self.ui.positionMarginSpin, 'position_margin'),
                (self.ui.bodyZOffsetSpin, 'body_z_offset_mm'),
                (self.ui.staticCenterSpin, 'static_center_factor'),
                (self.ui.staticStableSpin, 'static_stable_factor'),
                (self.ui.staticMarginSpin, 'static_margin_mm'),
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

        if config.has_option('gaitconfig', 'static_enable'):
            self.ui.staticEnableCheck.setChecked(
                config.getboolean('gaitconfig', 'static_enable'))

        if config.has_option('gaitconfig', 'leg_order'):
            self.ui.legOrderEdit.setText(self.validate_leg_order(
                    config.get('gaitconfig', 'leg_order')))
        else:
            self.ui.legOrderEdit.setText(self.validate_leg_order(''))

        self.command_widget.read_settings(config)

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

        self.command_widget.write_settings(config)

        config.add_section('gaitconfig.legs')
        for leg_data in self.ripple_config.mechanical.leg_config.iteritems():
            leg_num, leg_config = leg_data
            config.set('gaitconfig.legs', 'leg.%d' % leg_num,
                       self.leg_config_to_string(leg_config))

        config.set('gaitconfig', 'static_enable',
                   self.ui.staticEnableCheck.isChecked())

        config.set('gaitconfig', 'leg_order',
                   self.validate_leg_order(self.ui.legOrderEdit.text()))

    def intify_leg_order(self, data):
        result = []
        if data == '':
            return result

        in_tuple = False
        current_tuple = ()
        current_item = ''
        for x in data:
            if x == '(':
                if in_tuple:
                    return result
                in_tuple = True

            if x >= '0' and x <= '9':
                current_item += x
            else:
                if len(current_item):
                    value = int(current_item)
                    current_item = ''
                    if in_tuple:
                        current_tuple += (value,)
                    else:
                        result.append(value)
                if x == ')':
                    if not in_tuple:
                        return result
                    if len(current_tuple) == 1:
                        result.append(current_tuple[0])
                    elif len(current_tuple) > 1:
                        result.append(current_tuple)
                    current_tuple = ()
                    in_tuple = False

        if len(current_item):
            result.append(int(current_item))

        return result

    def stringify_leg_order(self, data):
        assert isinstance(data, list)
        return str(data)[1:-1].replace(' ', '')

    def validate_leg_order(self, data):
        """Accept data that is human entered.  Return a string
        containing a valid leg ordering, which will consist of a comma
        separate list of leg numbers, or tuples of leg numbers."""
        entered_values = []
        try:
            entered_values = self.intify_leg_order(data)
        except:
            pass

        required_legs = self.ripple_config.mechanical.leg_config.keys()

        used_legs = {}
        actual_legs = []
        for group in entered_values:
            if isinstance(group, int):
                x = group
                if x in required_legs and not x in used_legs:
                    actual_legs.append(x)
                    used_legs[x] = True
            else:
                next_tuple = ()
                for x in group:
                    if x in required_legs and not x in used_legs:
                        next_tuple += (x,)
                        used_legs[x] = True
                if len(next_tuple) > 1:
                    actual_legs.append(next_tuple)
                elif len(next_tuple) == 1:
                    actual_legs.append(next_tuple[0])

        for x in required_legs:
            if not x in used_legs:
                actual_legs.append(x)

        return self.stringify_leg_order(actual_legs)

    def handle_current_changed(self, index=2):
        if index != 2:
            # Make sure we're not still playing.
            self.ui.playbackSingleButton.setChecked(False)
            self.ui.playbackRepeatButton.setChecked(False)
            self.ui.playbackSlowRepeatButton.setChecked(False)

            if self.servo_tab.controller:
                Task(self.servo_tab.controller.enable_power(
                        servo_controller.POWER_BRAKE))
            return

        if self.servo_tab.controller:
            Task(self.servo_tab.controller.enable_power(
                    servo_controller.POWER_ENABLE))

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

        self.command_widget.fit_in_view()

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
        self.ripple_config.swing_percent = \
            self.ui.swingPercentSpin.value()
        self.ripple_config.position_margin_percent = \
            self.ui.positionMarginSpin.value()

        self.ripple_config.leg_order = self.intify_leg_order(
            self.validate_leg_order(self.ui.legOrderEdit.text()))

        self.ripple_config.body_z_offset_mm = self.ui.bodyZOffsetSpin.value()

        self.ripple_config.statically_stable = \
            self.ui.staticEnableCheck.isChecked()
        self.ripple_config.static_center_factor = \
            self.ui.staticCenterSpin.value()
        self.ripple_config.static_stable_factor = \
            self.ui.staticStableSpin.value()
        self.ripple_config.static_margin_mm = \
            self.ui.staticMarginSpin.value()

        self.ripple_gait = ripple_gait.RippleGait(self.ripple_config)
        self.gait_geometry_display.set_gait_config(self.ripple_config)

        self.update_gait_graph()
        self.handle_playback_config_change()
        self.update_allowable_commands()

    def handle_leg_order_editing_finished(self):
        self.ui.legOrderEdit.setText(self.validate_leg_order(
                self.ui.legOrderEdit.text()))
        self.handle_gait_config_change()

    def update_gait_graph(self):
        self.gait_graph_display.set_gait_graph(self.ripple_gait.get_gait_graph())

    def get_start_state(self):
        begin_index = self.ui.playbackBeginCombo.currentIndex()
        this_ripple = ripple_gait.RippleGait(self.ripple_config)

        if begin_index == 0: # Idle
            begin_state = this_ripple.get_idle_state()
        else:
            begin_state = this_ripple.get_idle_state()
            this_ripple.set_state(begin_state, self.command)
            for x in range(int(1.0 / self.phase_step)):
                begin_state = this_ripple.advance_phase(self.phase_step)

            # When setting a state, we are required to be exactly
            # zero.  Verify that we are close enough to zero from a
            # numerical perspective, then force it to be exactly zero.
            assert abs(((begin_state.phase + 0.5) % 1.0) - 0.5) < 1e-4
            begin_state.phase = 0.0

        return begin_state

    def handle_playback_config_change(self):
        # Re-run the playback recording the state through an entire
        # phase.  Then make sure that the graphic state is current for
        # the phase that is selected now.

        try:
            begin_state = self.get_start_state()
            begin_state = self.ripple_gait.set_state(begin_state, self.command)
        except ripple_gait.NotSupported:
            # guess we can't change anything
            self.ui.gaitOptionsBrowser.setText('command not possible')
            return

        self.current_states = (
            [begin_state.copy()] +
            [self.ripple_gait.advance_phase(self.phase_step).copy()
             for x in range(self.ui.playbackPhaseSlider.maximum())])

        self.handle_playback_phase_change()

        options = self.ripple_gait.options
        text = 'cycle_time: %.2fs\nservo_speed: %.1fdps' % (
            options.cycle_time_s,
            options.servo_speed_dps)
        self.ui.gaitOptionsBrowser.setText(text)

    def handle_playback_phase_change(self):
        if self.playback_mode != self.PLAYBACK_IDLE:
            return
        self.update_phase(self.ui.playbackPhaseSlider.value() * self.phase_step)

    def update_phase(self, phase):
        # Update the current geometry rendering.
        state = self.current_states[int(phase / self.phase_step)]
        self.render_state(state)

    def render_state(self, state):
        # Render the phase line in the gait graph.
        self.gait_graph_display.set_phase(state.phase % 1.0)
        self.gait_geometry_display.set_state(state)

        if self.servo_tab.controller:
            if (self.current_command is not None and
                not self.current_command.done()):
                self.skipped_update += 1
                return

            try:
                command = state.command_dict()
            except ripple_gait.NotSupported:
                return

            self.current_command = Task(
                self.servo_tab.controller.set_pose(
                    command,
                    pose_time=2 * 0.001 * PLAYBACK_TIMEOUT_MS))

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
            for spin, name in self.command_spins:
                setattr(self.command, name, spin.value())

            self.update_command()
            self.update_allowable_commands()

    def update_allowable_commands(self):
        self.command_widget.update_allowable(self.ripple_config, self.command)

    def handle_command_reset(self):
        with self.in_command_changed:
            for spin, _ in self.command_spins:
                spin.setValue(0.0)

        self.handle_command_change()

    def handle_playback_state_change(self, state, checked):
        if not checked:
            # If nothing is checked, then stop playback.
            if (not self.ui.playbackSingleButton.isChecked() and
                not self.ui.playbackRepeatButton.isChecked() and
                not self.ui.playbackSlowRepeatButton.isChecked()):

                self.playback_timer.stop()
                self.playback_mode = self.PLAYBACK_IDLE

            self.handle_playback_config_change()

            return

        # Make sure everything else is unchecked.
        if state != self.PLAYBACK_SINGLE:
            self.ui.playbackSingleButton.setChecked(False)
        if state != self.PLAYBACK_REPEAT:
            self.ui.playbackRepeatButton.setChecked(False)
        if state != self.PLAYBACK_SLOW_REPEAT:
            self.ui.playbackSlowRepeatButton.setChecked(False)

        # Otherwise, start the appropriate playback mode.
        self.ripple_gait.set_state(self.get_start_state(), self.command)
        self.playback_mode = state
        self.playback_timer.start(PLAYBACK_TIMEOUT_MS)

    def handle_playback_timer(self):
        if self.playback_mode == self.PLAYBACK_IDLE:
            print "WARNING: Playback timer fired when idle."
            return

        old_phase = self.ripple_gait.state.phase

        advance = PLAYBACK_TIMEOUT_MS / 1000.0
        if self.playback_mode == self.PLAYBACK_SLOW_REPEAT:
            advance *= 0.1
        state = self.ripple_gait.advance_time(advance)

        if (self.playback_mode == self.PLAYBACK_SINGLE and
            state.phase < 0.5 and old_phase > 0.5):
            self.ui.playbackSingleButton.setChecked(False)
            return

        self.render_state(state)

    def handle_widget_set_command(self):
        with self.in_command_changed:
            for spin, name in self.command_spins:
                spin.setValue(getattr(self.command, name))

            self.update_command()

    def update_command(self):
        if self.playback_mode == self.PLAYBACK_IDLE:
            # If we're idle, we can just update the phase list right away.
            self.handle_playback_config_change()
        else:
            # Otherwise, just set the command on our gait and let
            # playback do its thing.
            self.ripple_gait.set_command(self.command)
