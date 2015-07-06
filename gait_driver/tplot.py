#!/usr/bin/env python
# Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.

import capnp
import datetime
import sys
import time

import matplotlib

matplotlib.use('Qt4Agg')
matplotlib.rcParams['backend.qt4'] = 'PySide'

from matplotlib.backends import backend_qt4agg
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

import ui_tplot_main_window

import telemetry_log

class BoolGuard(object):
    def __init__(self):
        self.value = False

    def __enter__(self):
        self.value = True

    def __exit__(self, type, value, traceback):
        self.value = False

    def active(self):
        return self.value


class Log(object):
    def __init__(self, filename):
        self.reader = telemetry_log.BulkReader(open(filename))
        self.records = self.reader.records()
        self.all = None

    def get_all(self):
        if self.all:
            return
        self.all = self.reader.get()


def _bisect(array, item, key):
    if len(array) == 0:
        return None
    if item < key(array[0]):
        return None

    lower = 0
    upper = len(array)

    while abs(lower - upper) > 1:
        mid = (lower + upper) / 2
        value = key(array[mid])
        if item < value:
            upper = mid
        else:
            lower = mid

    return lower


def _clear_tree_widget(item):
    item.setText(1, '')
    for i in range(item.childCount()):
        child = item.child(i)
        _clear_tree_widget(child)


def _set_tree_widget_data(item, struct):
    for i in range(item.childCount()):
        child = item.child(i)
        name = child.text(0)

        field = getattr(struct, name)
        if isinstance(field, capnp.lib.capnp._DynamicStructReader):
            _set_tree_widget_data(child, field)
        else:
            child.setText(1, str(field))


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

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.handle_timeout)

        self.time_start = None
        self.time_end = None
        self.time_current = None

        self.ui.recordCombo.currentIndexChanged.connect(
            self.handle_record_combo)
        self.ui.addPlotButton.clicked.connect(self.handle_add_plot_button)
        self.ui.removeButton.clicked.connect(self.handle_remove_button)
        self.ui.treeWidget.itemExpanded.connect(self.handle_item_expanded)
        self.tree_items = []
        self.ui.treeWidget.header().setResizeMode(
            QtGui.QHeaderView.ResizeToContents)
        self.ui.timeSlider.valueChanged.connect(self.handle_time_slider)
        self._updating_slider = BoolGuard()

        self.ui.fastReverseButton.clicked.connect(
            self.handle_fast_reverse_button)
        self.ui.stepBackButton.clicked.connect(
            self.handle_step_back_button)
        self.ui.playReverseButton.clicked.connect(
            self.handle_play_reverse_button)
        self.ui.stopButton.clicked.connect(self.handle_stop_button)
        self.ui.playButton.clicked.connect(self.handle_play_button)
        self.ui.stepForwardButton.clicked.connect(
            self.handle_step_forward_button)
        self.ui.fastForwardButton.clicked.connect(
            self.handle_fast_forward_button)

    def open(self, filename):
        try:
            maybe_log = Log(filename)
        except Exception as e:
            QtGui.QMessageBox.warning(self, 'Could not open log',
                                      'Error: ' + str(e))
            return

        # OK, we're good, clear out our UI.
        self.ui.treeWidget.clear()
        self.tree_items = []
        self.ui.recordCombo.clear()
        self.ui.xCombo.clear()
        self.ui.yCombo.clear()

        self.log = maybe_log
        for name in self.log.records.keys():
            self.ui.recordCombo.addItem(name)

            item = QtGui.QTreeWidgetItem()
            item.setText(0, name)
            self.ui.treeWidget.addTopLevelItem(item)
            self.tree_items.append(item)

            exemplar = self.log.records[name]
            def add_item(parent, element):
                for name in sorted(dir(element)):
                    item = QtGui.QTreeWidgetItem(parent)
                    item.setText(0, name)

                    child = getattr(element, name)
                    if isinstance(child, capnp.lib.capnp._DynamicStructReader):
                        add_item(item, child)
            add_item(item, exemplar)

    def handle_record_combo(self):
        record = self.ui.recordCombo.currentText()
        self.ui.xCombo.clear()
        self.ui.yCombo.clear()

        exemplar = self.log.records[record]
        default_x = None
        for index, name in enumerate(sorted(dir(exemplar))):
            self.ui.xCombo.addItem(name)
            self.ui.yCombo.addItem(name)

            if name == 'timestamp':
                default_x = index

        if default_x:
            self.ui.xCombo.setCurrentIndex(default_x)

    def handle_add_plot_button(self):
        self.log.get_all()

        record = self.ui.recordCombo.currentText()
        xname = self.ui.xCombo.currentText()
        yname = self.ui.yCombo.currentText()

        data = self.log.all[record]
        xdata = [getattr(x, xname) for x in data]
        ydata = [getattr(x, yname) for x in data]

        line = matplotlib.lines.Line2D(xdata, ydata)
        line.tplot_record_name = record
        line.tplot_xname = xname
        line.tplot_yname = yname
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
        if hasattr(line, 'tplot_marker'):
            line.tplot_marker.remove()
        line.remove()
        self.ui.plotsCombo.removeItem(index)

        self.canvas.draw()

    def handle_item_expanded(self):
        self.update_timeline()

    def update_timeline(self):
        if self.time_start is not None:
            return

        self.log.get_all()

        # Look through all the records for those which have a
        # "timestamp" field.  Find the minimum and maximum of each.
        for record, exemplar in self.log.records.iteritems():
            if not hasattr(exemplar, 'timestamp'):
                continue

            these_times = [x.timestamp for x in self.log.all[record]]
            this_min = min(these_times)
            this_max = max(these_times)

            if self.time_start is None or this_min < self.time_start:
                self.time_start = this_min
            if self.time_end is None or this_max > self.time_end:
                self.time_end = this_max

        self.time_current = self.time_start
        self.update_time(self.time_current, update_slider=False)

    def handle_mouse(self, event):
        if not event.inaxes:
            return
        self.statusBar().showMessage('%f,%f' % (event.xdata, event.ydata))

    def update_time(self, new_time, update_slider=True):
        new_time = max(self.time_start, min(self.time_end, new_time))
        self.time_current = new_time

        # Update the tree view.
        self.update_tree_view(new_time)

        # Update dots on the plot.
        self.update_plot_dots(new_time)

        # Update the text fields.
        dt = datetime.datetime.utcfromtimestamp(new_time)
        self.ui.clockEdit.setText('%04d-%02d-%02d %02d:%02d:%02.3f' % (
                dt.year, dt.month, dt.day,
                dt.hour, dt.minute, dt.second + dt.microsecond / 1e6))
        self.ui.elapsedEdit.setText('%.3f' % (new_time - self.time_start))

        if update_slider:
            with self._updating_slider:
                elapsed = new_time - self.time_start
                total_time = self.time_end - self.time_start
                self.ui.timeSlider.setValue(
                    int(1000 * elapsed / total_time))

    def handle_time_slider(self):
        if self._updating_slider.active():
            return

        total_time = self.time_end - self.time_start
        current = self.ui.timeSlider.value() / 1000.0
        self.update_time(self.time_start + current * total_time,
                         update_slider=False)

    def update_tree_view(self, time):
        for item in self.tree_items:
            name = item.text(0)
            all_data = self.log.all[name]

            this_data_index = _bisect(all_data, time,
                                      key=lambda x: x.timestamp)
            if this_data_index is None:
                _clear_tree_widget(item)
            else:
                this_data = all_data[this_data_index]
                _set_tree_widget_data(item, this_data)

    def update_plot_dots(self, new_time):
        updated = False
        for line in self.left_axis.lines:
            if not hasattr(line, 'tplot_record_name'):
                continue
            if not hasattr(self.log.records[line.tplot_record_name],
                           'timestamp'):
                continue

            all_data = self.log.all[line.tplot_record_name]
            this_index = _bisect(all_data, new_time, lambda x: x.timestamp)
            if this_index is None:
                continue

            this_data = all_data[this_index]

            if not hasattr(line, 'tplot_marker'):
                line.tplot_marker = matplotlib.lines.Line2D([], [])
                line.tplot_marker.set_marker('o')
                line.tplot_marker.set_color(line._color)
                self.left_axis.add_line(line.tplot_marker)

            updated = True
            xdata = [getattr(this_data, line.tplot_xname)]
            ydata = [getattr(this_data, line.tplot_yname)]
            line.tplot_marker.set_data(xdata, ydata)

        if updated:
            self.canvas.draw()


    def handle_fast_reverse_button(self):
        self.play_start(-self.ui.fastReverseSpin.value())

    def handle_step_back_button(self):
        self.play_stop()
        self.update_time(self.time_current - self.ui.stepBackSpin.value())

    def handle_play_reverse_button(self):
        self.play_start(-1.0)

    def handle_stop_button(self):
        self.play_stop()

    def handle_play_button(self):
        self.play_start(1.0)

    def handle_step_forward_button(self):
        self.play_stop()
        self.update_time(self.time_current + self.ui.stepForwardSpin.value())

    def handle_fast_forward_button(self):
        self.play_start(self.ui.fastForwardSpin.value())

    def play_stop(self):
        self.speed = None
        self.last_time = None
        self.timer.stop()

    def play_start(self, speed):
        self.speed = speed
        self.last_time = time.time()
        self.timer.start(100)

    def handle_timeout(self):
        assert self.last_time is not None
        this_time = time.time()
        delta_t = this_time - self.last_time
        self.last_time = this_time

        self.update_time(self.time_current + delta_t * self.speed)


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
