#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

'''A servo, inverse-kinematics, and gait configuration tool for
walking mech robots.'''

import argparse
import os
import logging
import sys
import traceback

import ConfigParser

import trollius
import trollius as asyncio

# The following is an ugly ugly hack to get useful stack traces from
# coroutines with trollius on python 2.
old_future_set = trollius.Future.set_exception

def Future_set_exception(self, exc):
    tb = sys.exc_info()[2]
    if not hasattr(exc, '__frames__'):
        setattr(exc, '__frames__', [])

    frames = getattr(exc, '__frames__')
    if len(frames) == 0:
        frames.append(str(exc))
    frames[0:0] = traceback.format_tb(tb)

    old_future_set(self, exc)

    self._tb_logger.tb = frames

    self = None

trollius.Future.set_exception = Future_set_exception


import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

import asyncio_qt
import mtool_main_window
import servo_controller

from mtool_servo_tab import ServoTab
from mtool_ik_config_tab import IkConfigTab
from mtool_gait_tab import GaitTab

class Mtool(QtGui.QMainWindow):
    DEFAULT_CONFIG_FILE = os.path.expanduser('~/.config/mtool/mtool.ini')

    def __init__(self, args, parent=None):
        super(Mtool, self).__init__(parent)

        self.config_file = (args.config if args.config else
                            self.DEFAULT_CONFIG_FILE)

        self.ui = mtool_main_window.Ui_MtoolMainWindow()
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
    app.setApplicationName('mtool')

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-c', '--config',
                        help='use a non-default configuration file')

    args = parser.parse_args()

    mtool = Mtool(args)
    mtool.show()

    loop = asyncio.get_event_loop()
    loop.run_forever()

if __name__ == '__main__':
    main()
