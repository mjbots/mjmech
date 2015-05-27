#!/usr/bin/python

import argparse
import logging
import sys

import trollius
import trollius as asyncio

sys.path.append('../../legtool/')

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

from legtool.async import asyncio_qt

import ui_manager_main_window


class ManagerMainWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(ManagerMainWindow, self).__init__(parent)

        self.ui = ui_manager_main_window.Ui_ManagerMainWindow()
        self.ui.setupUi(self)


def main():
    logging.basicConfig(level=logging.WARN, stream=sys.stdout)

    asyncio.set_event_loop_policy(asyncio_qt.QtEventLoopPolicy())

    app = QtGui.QApplication(sys.argv)
    app.setApplicationName('mjscore_manager')

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-s', '--serial',
                        help='serial port to use')

    args = parser.parse_args()

    manager = ManagerMainWindow()
    manager.show()

    asyncio.get_event_loop().run_forever()


if __name__ == '__main__':
    main()
