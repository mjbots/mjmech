#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

'''A servo, inverse-kinematics, and gait configuration tool for
walking mech robots.'''

import argparse
import os
import logging
import sys

import ConfigParser

import trollius
import trollius as asyncio

from legtool.async import trollius_trace

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

from legtool.async import asyncio_qt

import ui_legtool_main_window

from legtool.tabs.servo_tab import ServoTab
from legtool.tabs.ik_config_tab import IkConfigTab
from legtool.tabs.gait_tab import GaitTab

class LegTool(QtGui.QMainWindow):
    DEFAULT_CONFIG_FILE = os.path.expanduser('~/.config/legtool/legtool.ini')

    def __init__(self, args, parent=None):
        super(LegTool, self).__init__(parent)

        self.config_file = (args.config if args.config else
                            self.DEFAULT_CONFIG_FILE)

        self.ui = ui_legtool_main_window.Ui_LegToolMainWindow()
        self.ui.setupUi(self)

        self.servo_tab = ServoTab(self.ui, self.statusBar())
        self.ikconfig_tab = IkConfigTab(self.ui, self.servo_tab)
        self.gait_tab = GaitTab(self.ui, self.ikconfig_tab, self.servo_tab)

        self.tabs = [self.servo_tab, self.ikconfig_tab, self.gait_tab]

        self.read_settings()

    def closeEvent(self, event):
        self.write_settings()
        event.accept()

    def read_settings(self):
        config = ConfigParser.ConfigParser()
        config.read(self.config_file)

        for tab in self.tabs:
            tab.read_settings(config)

    def write_settings(self):
        config = ConfigParser.ConfigParser()

        for tab in self.tabs:
            tab.write_settings(config)

        config_dir = os.path.dirname(self.config_file)
        if config_dir != '' and not os.path.exists(config_dir):
            os.mkdir(config_dir)
        config.write(open(self.config_file, 'w'))

    def resizeEvent(self, event):
        for tab in self.tabs:
            tab.resizeEvent(event)

def main():
    logging.basicConfig(level=logging.WARN, stream=sys.stdout)

    asyncio.set_event_loop_policy(asyncio_qt.QtEventLoopPolicy())

    app = QtGui.QApplication(sys.argv)
    app.setApplicationName('legtool')

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-c', '--config',
                        help='use a non-default configuration file')

    args = parser.parse_args()

    legtool = LegTool(args)
    legtool.show()

    loop = asyncio.get_event_loop()
    loop.run_forever()

if __name__ == '__main__':
    main()
