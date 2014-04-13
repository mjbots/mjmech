# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

class GaitTab(object):
    def __init__(self, ui, ikconfig_tab, servo_tab):
        self.ui = ui
        self.ikconfig_tab = ikconfig_tab
        self.servo_tab = servo_tab

        self.ui.tabWidget.currentChanged.connect(self.handle_current_changed)
        self.handle_current_changed()

    def handle_current_changed(self, index = 2):
        if index != 2:
            return

        # TODO jpieper: Ensure we copy all necessary information from
        # the ikconfig tab... (likely just the count and numbers of
        # the present legs).
