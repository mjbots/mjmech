# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

import ripple_gait

from mtool_common import BoolContext

class GaitTab(object):
    def __init__(self, ui, ikconfig_tab, servo_tab):
        self.ui = ui
        self.ikconfig_tab = ikconfig_tab
        self.servo_tab = servo_tab

        self.in_number_changed = BoolContext()

        self.ripple_config = ripple_gait.RippleConfig()

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
        self.handle_current_changed()

    def read_settings(self, settings):
        pass

    def write_settings(self, settings):
        pass

    def handle_current_changed(self, index = 2):
        if index != 2:
            return

        # TODO jpieper: Ensure we copy all necessary information from
        # the ikconfig tab... (likely just the count and numbers of
        # the present legs).

        available_legs = self.ikconfig_tab.get_all_legs()
        enabled_legs = set(self.ikconfig_tab.get_enabled_legs())

        # Update the leg list widget.
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
        # TODO jpieper
        pass
