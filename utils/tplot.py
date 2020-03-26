#!/usr/bin/python3 -B

# Copyright 2015-2020 Josh Pieper, jjp@pobox.com.
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

import datetime
import os
import sys
import time

import matplotlib

matplotlib.use('Qt4Agg')
matplotlib.rcParams['backend.qt4'] = 'PySide'

from matplotlib.backends import backend_qt4agg
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

SCRIPT_PATH=os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(SCRIPT_PATH, '../python'))
sys.path.append(os.path.join(SCRIPT_PATH, '../bazel-bin/utils'))
import ui_tplot_main_window

import mjlib.telemetry.reader as reader
import mjlib.telemetry.file_reader as file_reader


AXES = ['Left', 'Right', '3', '4']

LEGEND_LOC = {
    'Left': 2,
    'Right': 1,
    '3': 7,
    '4': 4
    }


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
        self.reader = file_reader.FileReader(filename)
        self.records = self.reader.records()
        self.all = None

    def get_all(self):
        if self.all is not None:
            return
        self.all = self.reader.get()


def _walk_item(item):
    fields = None
    while item.parent():
        fields = item.text(0) + ("." + fields if fields else "")
        item = item.parent()
    return item.text(0), fields


def _add_schema_struct_to_tree_view(parent_item, schema):
    for field in schema.fields:
        name = field.name

        item = QtGui.QTreeWidgetItem(parent_item)
        item.setText(0, name)

        _add_schema_item_to_tree_view(item, field.type_class)


def _add_schema_item_to_tree_view(item, schema):
    if isinstance(schema, reader.ObjectType):
        _add_schema_struct_to_tree_view(item, schema)
    else:
        # All other types we ignore for now, and let them get filled
        # in on the first data.
        pass


def _make_timestamp_getter(all_data):
    if len(all_data) == 0:
        return lambda x: 0.0
    sample = all_data[0]

    # If any children have a timestamp field, use the first one we can
    # find.
    def find_child(prefix, value):
        if hasattr(value, 'timestamp'):
            return lambda x: _get_data(x, prefix + 'timestamp')
        if not hasattr(value, '_fields'):
            return None
        for child in value._fields:
            result = find_child(prefix + child + '.', getattr(value, child))
            if result:
                return result
        return None

    return find_child('', sample)


def _bisect(array, item, key):
    if len(array) == 0:
        return None
    if item < key(array[0]):
        return None

    lower = 0
    upper = len(array)

    while abs(lower - upper) > 1:
        mid = (lower + upper) // 2
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


def _set_tree_widget_data_struct(qt_item, data_struct, schema):
    for schema_index, field in enumerate(schema.fields):
        name = field.name
        data_field = getattr(data_struct, name)

        _set_tree_widget_data_item(
            qt_item.child(schema_index), data_field, field.type_class)


def _set_tree_widget_data_item(qt_item, data_struct, schema):
    if qt_item is None:
        # TODO(jpieper): I have no idea why this is necessary right
        # now.
        return
    if isinstance(schema, reader.ObjectType):
        # This field is a structure.
        _set_tree_widget_data_struct(qt_item, data_struct, schema)
    elif isinstance(schema, reader.UnionType):
        # We are likely an optional.
        if data_struct is None:
            _clear_tree_widget(qt_item)
        else:
            assert len(schema.items) == 2
            assert isinstance(schema.items[0], reader.NullType)
            _set_tree_widget_data_item(qt_item, data_struct, schema.items[1])
    elif isinstance(schema, reader.ArrayType):
        assert isinstance(data_struct, list)
        for i in range(qt_item.childCount(), len(data_struct)):
            subitem = QtGui.QTreeWidgetItem(qt_item)
            subitem.setText(0, str(i))
            _add_schema_item_to_tree_view(subitem, schema.type_class)
        for i in range(len(data_struct)):
            subitem = qt_item.child(i)
            _set_tree_widget_data_item(
                subitem, data_struct[i], schema.type_class)
        while qt_item.childCount() > len(data_struct):
            qt_item.removeChild(qt_item.child(qt_item.childCount() - 1))

    else:
        # We must just be text!
        qt_item.setText(1, str(data_struct))


def _get_data(value, name):
    fields = name.split('.')
    for field in fields:
        if isinstance(value, list):
            try:
                value = value[int(field)]
            except IndexError:
                return None
        else:
            value = getattr(value, field)
    return value


class Tplot(QtGui.QMainWindow):
    def __init__(self):
        super(Tplot, self).__init__()

        self.ui = ui_tplot_main_window.Ui_TplotMainWindow()
        self.ui.setupUi(self)

        self.figure = matplotlib.figure.Figure()
        self.canvas = FigureCanvas(self.figure)

        self.canvas.mpl_connect('motion_notify_event', self.handle_mouse)
        self.canvas.mpl_connect('key_press_event', self.handle_key_press)
        self.canvas.mpl_connect('key_release_event', self.handle_key_release)

        # Make QT drawing not be super slow.  See:
        # https://github.com/matplotlib/matplotlib/issues/2559/
        def draw():
            FigureCanvas.draw(self.canvas)
            self.canvas.repaint()

        self.canvas.draw = draw

        self.left_axis = self.figure.add_subplot(111)
        self.left_axis.tplot_name = 'Left'

        self.axes = {
            'Left' : self.left_axis,
            }

        layout = QtGui.QVBoxLayout(self.ui.plotFrame)
        layout.addWidget(self.canvas, 1)

        self.toolbar = backend_qt4agg.NavigationToolbar2QT(self.canvas, self)
        self.addToolBar(self.toolbar)

        self.canvas.setFocusPolicy(QtCore.Qt.ClickFocus)
        self.canvas.setFocus()

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
        self.ui.treeWidget.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.ui.treeWidget.customContextMenuRequested.connect(self.handle_tree_widget_context_menu)
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
            raise
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

            schema = self.log.records[name]
            _add_schema_struct_to_tree_view(item, schema)

    def handle_record_combo(self):
        record = self.ui.recordCombo.currentText()
        self.ui.xCombo.clear()
        self.ui.yCombo.clear()

        schema = self.log.records[record]
        default_x = None
        index = [0, None]

        def add_item(index, parent, schema):
            if not isinstance(schema, reader.ObjectType):
                return
            for field in schema.fields:
                name = field.name
                this_name = parent
                if len(this_name):
                    this_name += '.'
                this_name += name

                if isinstance(field.type_class, reader.ArrayType):
                    child = field.type_class.type_class
                    # TODO: Figure out how many to add to the record
                    # drop down.
                    pass
                elif isinstance(field.type_class, reader.ObjectType):
                    for field in field.type_class.fields:
                        add_item(index, this_name, field.type_class)
                else:
                    self.ui.xCombo.addItem(this_name)
                    self.ui.yCombo.addItem(this_name)

                    if name == 'timestamp':
                        index[1] = index[0]

                    index[0] += 1

        add_item(index, '', schema)
        default_x = index[1]

        if default_x:
            self.ui.xCombo.setCurrentIndex(default_x)

    def handle_add_plot_button(self):
        record = self.ui.recordCombo.currentText()
        xname = self.ui.xCombo.currentText()
        yname = self.ui.yCombo.currentText()

        self._add_plot(record, xname, yname)

    def _add_plot(self, record, xname, yname):
        self.log.get_all()

        data = self.log.all[record]
        xdata = [_get_data(x.data, xname) for x in data]
        ydata = [_get_data(x.data, yname) for x in data]

        line = matplotlib.lines.Line2D(xdata, ydata)
        line.tplot_record_name = record
        if 'timestamp' in [x.name for x in self.log.records[record].fields]:
            line.tplot_has_timestamp = True
        line.tplot_xname = xname
        line.tplot_yname = yname
        label = self.make_label(record, xname, yname)
        line.set_label(label)
        line.set_color(self.COLORS[self.next_color])
        self.next_color = (self.next_color + 1) % len(self.COLORS)

        axis = self.get_current_axis()

        axis.add_line(line)
        axis.relim()
        axis.autoscale_view()
        axis.legend(loc=LEGEND_LOC[axis.tplot_name])

        self.ui.plotsCombo.addItem(label, line)
        self.ui.plotsCombo.setCurrentIndex(self.ui.plotsCombo.count() - 1)

        self.canvas.draw()

    def make_label(self, record, xname, yname):
        if xname == 'timestamp':
            return '%s.%s' % (record, yname)
        return '%s %s vs. %s' % (record, yname, xname)

    def get_current_axis(self):
        requested = self.ui.axisCombo.currentText()
        maybe_result = self.axes.get(requested, None)
        if maybe_result:
            return maybe_result

        result = self.left_axis.twinx()
        self.axes[requested] = result
        result.tplot_name = requested

        return result

    def get_all_axes(self):
        return self.axes.values()

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

    def handle_tree_widget_context_menu(self, point):
        item = self.ui.treeWidget.itemAt(point)
        record, fields = _walk_item(item)

        if record is None or fields is None:
            return

        menu = QtGui.QMenu()
        plot_vs_time = menu.addAction("Plot vs time")
        result = menu.exec_(self.ui.treeWidget.mapToGlobal(point))
        if result is None:
            return
        if result == plot_vs_time:
            self._add_plot(record, 'timestamp', fields)


    def update_timeline(self):
        if self.time_start is not None:
            return

        self.log.get_all()

        # Look through all the records for those which have a
        # "timestamp" field.  Find the minimum and maximum of each.
        for record_name, schema in self.log.records.items():
            if record_name not in self.log.all:
                continue
            timestamp_getter = _make_timestamp_getter(self.log.all[record_name])
            if timestamp_getter is None:
                continue

            these_times = [timestamp_getter(x) for x in self.log.all[record_name]]
            if len(these_times) == 0:
                continue
            this_min = min(these_times)
            this_max = max(these_times)

            if this_min < 0 or this_max < 0:
                continue

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

    def handle_key_press(self, event):
        if event.key not in ['1', '2', '3', '4']:
            return
        index = ord(event.key) - ord('1')
        for key, value in self.axes.items():
            if key == AXES[index]:
                value.set_navigate(True)
            else:
                value.set_navigate(False)

    def handle_key_release(self, event):
        if event.key not in ['1', '2', '3', '4']:
            return
        for key, value in self.axes.items():
            value.set_navigate(True)

    def update_time(self, new_time, update_slider=True):
        if new_time is None:
            return
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

        if self.time_end is None or self.time_start is None:
            return

        total_time = self.time_end - self.time_start
        current = self.ui.timeSlider.value() / 1000.0
        self.update_time(self.time_start + current * total_time,
                         update_slider=False)

    def update_tree_view(self, time):
        for item in self.tree_items:
            name = item.text(0)
            if name not in self.log.all:
                continue
            all_data = self.log.all[name]

            timestamp_getter = _make_timestamp_getter(all_data)
            if timestamp_getter is None:
                continue

            this_data_index = _bisect(all_data, time, key=timestamp_getter)
            if this_data_index is None:
                _clear_tree_widget(item)
            else:
                this_data = all_data[this_data_index]
                _set_tree_widget_data_struct(item, this_data.data,
                                             self.log.records[name])

    def update_plot_dots(self, new_time):
        updated = False
        for axis in self.get_all_axes():
            for line in axis.lines:
                if not hasattr(line, 'tplot_record_name'):
                    continue
                if not hasattr(line, 'tplot_has_timestamp'):
                    continue

                all_data = self.log.all[line.tplot_record_name]
                timestamp_getter = _make_timestamp_getter(all_data)
                this_index = _bisect(all_data, new_time, timestamp_getter)
                if this_index is None:
                    continue

                this_data = all_data[this_index]

                if not hasattr(line, 'tplot_marker'):
                    line.tplot_marker = matplotlib.lines.Line2D([], [])
                    line.tplot_marker.set_marker('o')
                    line.tplot_marker.set_color(line._color)
                    axis.add_line(line.tplot_marker)

                updated = True
                xdata = [_get_data(this_data.data, line.tplot_xname)]
                ydata = [_get_data(this_data.data, line.tplot_yname)]
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
