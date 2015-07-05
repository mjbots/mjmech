#!/usr/bin/env python
# Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.

import sys

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

import ui_tplot_main_window


class Tplot(QtGui.QMainWindow):
    def __init__(self):
        super(Tplot, self).__init__()

        self.ui = ui_tplot_main_window.Ui_TplotMainWindow()
        self.ui.setupUi(self)


def main():
    app = QtGui.QApplication(sys.argv)
    app.setApplicationName('tplot')

    tplot = Tplot()
    tplot.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
