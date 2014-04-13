# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

import ripple_gait

from mtool_common import BoolContext

class GaitGraphDisplay(object):
    def __init__(self, ui):
        self.graphics_scene = QtGui.QGraphicsScene()
        self.graphics_view = ui.gaitGraphView

        self.graphics_view.setScene(self.graphics_scene)

    def fit_in_view(self):
        self.graphics_view.fitInView(QtCore.QRectF(-0.1, 0, 1.1, 1))

    def set_gait_graph(self, graph):
        self.graphics_scene.clear()

        leg_numbers = sorted(graph.leg.keys())
        count = len(leg_numbers)

        if count == 0:
            return

        self.graphics_scene.addRect(0., 0., 1., 1.)

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

        self.ripple_config = ripple_gait.RippleConfig()
        self.ripple_gait = ripple_gait.RippleGait(self.ripple_config)

        self.gait_graph_display = GaitGraphDisplay(self.ui)

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
                     self.ui.maxSwingPercentSpin]:
            spin.valueChanged.connect(self.handle_gait_config_change)

        self.ui.tabWidget.currentChanged.connect(self.handle_current_changed)

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
                (self.ui.bodyZOffsetSpin, 'body_z_offset_mm')]

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

        if config.has_section('gaitconfig.legs'):
            for leg_name, value in config.items('gaitconfig.legs'):
                leg_num = int(leg_name.split('.')[1])
                self.ripple_config.mechanical.leg_config[leg_num] = \
                    self.string_to_leg_config(value)

        self.handle_leg_change(self.ui.gaitLegList.currentItem())
        self.handle_gait_config_change()

    def write_settings(self, config):
        config.add_section('gaitconfig')
        for spin, name in self.get_float_configs():
            config.set('gaitconfig', name, spin.value())

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

        self.update_gait_graph()

    def update_gait_graph(self):
        self.gait_graph_display.set_gait_graph(self.ripple_gait.get_gait_graph())
