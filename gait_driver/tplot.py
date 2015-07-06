#!/usr/bin/env python
# Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.

import sys

import matplotlib

matplotlib.use('Qt4Agg')
matplotlib.rcParams['backend.qt4'] = 'PySide'

from matplotlib.backends import backend_qt4agg
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

import ui_tplot_main_window

import telemetry_log

class Log(object):
    def __init__(self, filename):
        self.reader = telemetry_log.BulkReader(open(filename))
        self.records = self.reader.records()

    def get_all(self):
        self.all = self.reader.get()


class Tplot(QtGui.QMainWindow):
    def __init__(self):
        super(Tplot, self).__init__()

        self.ui = ui_tplot_main_window.Ui_TplotMainWindow()
        self.ui.setupUi(self)

        self.figure = matplotlib.figure.Figure()
        self.canvas = FigureCanvas(self.figure)

        self.canvas.mpl_connect('motion_notify_event', self.handle_mouse)

        # Make QT drawing not be super slow.  See:
        # https://github.com/matplotlib/matplotlib/issues/2559/
        def draw():
            FigureCanvas.draw(self.canvas)
            self.canvas.repaint()

        self.canvas.draw = draw

        self.left_axis = self.figure.add_subplot(111)
        self.left_axis.legend()

        layout = QtGui.QVBoxLayout(self.ui.plotFrame)
        layout.addWidget(self.canvas, 1)

        self.toolbar = backend_qt4agg.NavigationToolbar2QT(self.canvas, self)
        self.addToolBar(self.toolbar)

        self.log = None
        self.COLORS = 'rgbcmyk'
        self.next_color = 0

        self.ui.recordCombo.currentIndexChanged.connect(
            self.handle_record_combo)
        self.ui.addPlotButton.clicked.connect(self.handle_add_plot_button)
        self.ui.removeButton.clicked.connect(self.handle_remove_button)

    def open(self, filename):
        try:
            maybe_log = Log(filename)
        except Exception as e:
            QtGui.QMessageBox.warning(self, 'Could not open log',
                                      'Error: ' + str(e))
            return

        # OK, we're good, clear out our UI.
        self.ui.treeWidget.clear()
        self.ui.recordCombo.clear()
        self.ui.xCombo.clear()
        self.ui.yCombo.clear()

        self.log = maybe_log
        for name in self.log.records.keys():
            self.ui.recordCombo.addItem(name)

    def handle_record_combo(self):
        record = self.ui.recordCombo.currentText()
        self.ui.xCombo.clear()
        self.ui.yCombo.clear()

        exemplar = self.log.records[record]
        default_x = None
        for index, name in enumerate(dir(exemplar)):
            self.ui.xCombo.addItem(name)
            self.ui.yCombo.addItem(name)

            if name == 'timestamp':
                default_x = index

        if default_x:
            self.ui.xCombo.setCurrentIndex(default_x)

        # TODO jpieper: Update tree view.

    def handle_add_plot_button(self):
        self.log.get_all()

        record = self.ui.recordCombo.currentText()
        xname = self.ui.xCombo.currentText()
        yname = self.ui.yCombo.currentText()

        data = self.log.all[record]
        xdata = [getattr(x, xname) for x in data]
        ydata = [getattr(x, yname) for x in data]

        line = matplotlib.lines.Line2D(xdata, ydata)
        label = '%s %s vs. %s' % (record, xname, yname)
        line.set_label(label)
        line.set_color(self.COLORS[self.next_color])
        self.next_color = (self.next_color + 1) % len(self.COLORS)
        self.left_axis.add_line(line)
        self.left_axis.relim()
        self.left_axis.autoscale_view()

        self.left_axis.legend()

        self.ui.plotsCombo.addItem(label, line)
        self.ui.plotsCombo.setCurrentIndex(self.ui.plotsCombo.count() - 1)

        self.canvas.draw()

    def handle_remove_button(self):
        index = self.ui.plotsCombo.currentIndex()
        if index < 0:
            return
        line = self.ui.plotsCombo.itemData(index)
        line.remove()
        self.ui.plotsCombo.removeItem(index)

        self.canvas.draw()

    def handle_mouse(self, event):
        if not event.inaxes:
            return
        self.statusBar().showMessage('%f,%f' % (event.xdata, event.ydata))


def main():
    app = QtGui.QApplication(sys.argv)
    app.setApplicationName('tplot')

    tplot = Tplot()
    tplot.show()

    if len(sys.argv) > 0:
        tplot.open(sys.argv[1])

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
