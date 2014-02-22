#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

# TODO
#  * Implement single leg IK window
#  * Implement saving and restoring poses

import eventlet
import functools
import os
import sys

import ConfigParser

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

import mtool_main_window
import servo_controller

pool = None

def spawn(callback):
    def task():
        eventlet.spawn(callback)

    return task

class BoolContext(object):
    def __init__(self, ):
        self.value = False

    def __enter__(self):
        self.value = True

    def __exit__(self, type, value, traceback):
        self.value = False

class ServoTab(object):
    def __init__(self, ui):
        self.ui = ui

        self.servo_controls = []

        self.ui.statusText.setText('not connected')
        self.ui.connectButton.clicked.connect(
            spawn(self.handle_connect_clicked))

        servo_layout = QtGui.QVBoxLayout()
        servo_layout.setSpacing(0)
        servo_layout.setContentsMargins(0, 0, 0, 0)
        self.ui.scrollContents.setLayout(servo_layout)

        self.ui.servoCountSpin.valueChanged.connect(self.handle_servo_count)
        self.handle_servo_count()

        self.ui.powerCombo.currentIndexChanged.connect(
            spawn(self.handle_power))

        self.ui.captureCurrentButton.clicked.connect(
            spawn(self.handle_capture_current))

        self.update_connected(False)

        self.controller = None
        self.servo_update = BoolContext()

    def handle_connect_clicked(self):
        val = self.ui.typeCombo.currentText().lower()
        self.controller = servo_controller.servo_controller(
            val, serial_port=self.ui.serialPortCombo.currentText())
        self.ui.statusText.setText('connected')
        self.update_connected(True)

    def handle_servo_count(self):
        count = self.ui.servoCountSpin.value()
        while len(self.servo_controls) > count:
            # Remove the last one
            last = self.servo_controls[-1]
            widget = last['widget']
            self.ui.scrollContents.layout().removeWidget(widget)
            widget.deleteLater()
            self.servo_controls = self.servo_controls[:-1]

        while len(self.servo_controls) < count:
            # Add a new one.

            servo_id = len(self.servo_controls)

            label = QtGui.QLabel()
            label.setText('ID %d:' % servo_id)

            slider = QtGui.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(-180, 180)

            doublespin = QtGui.QDoubleSpinBox()
            doublespin.setRange(-180, 180)
            doublespin.setDecimals(1)

            widget = QtGui.QWidget()

            layout = QtGui.QHBoxLayout(widget)
            layout.addWidget(label)
            layout.addWidget(slider)
            layout.addWidget(doublespin)

            slider.valueChanged.connect(
                functools.partial(self.handle_servo_slider, servo_id))
            doublespin.valueChanged.connect(
                functools.partial(self.handle_servo_spin, servo_id))

            self.ui.scrollContents.layout().addWidget(widget)

            self.servo_controls.append({
                    'widget': widget,
                    'label': label,
                    'slider': slider,
                    'doublespin': doublespin})

    def handle_power(self):
        text = self.ui.powerCombo.currentText().lower()
        value = None
        if text == 'free':
            value = servo_controller.POWER_FREE
        elif text == 'brake':
            value = servo_controller.POWER_BRAKE
        elif text == 'drive':
            value = servo_controller.POWER_ENABLE
        else:
            raise NotImplementedError()

        self.controller.enable_power(value)

    def update_connected(self, value):
        self.ui.controlGroup.setEnabled(value)
        self.ui.posesGroup.setEnabled(value)
        if value:
            self.handle_power()

    def handle_servo_slider(self, servo_id, event):
        if self.servo_update.value:
            return

        with self.servo_update:
            control = self.servo_controls[servo_id]
            value = control['slider'].value()
            control['doublespin'].setValue(value)
            eventlet.spawn(self.controller.set_single_pose, servo_id, value)

    def handle_servo_spin(self, servo_id, event):
        if self.servo_update.value:
            return

        with self.servo_update:
            control = self.servo_controls[servo_id]
            value = control['doublespin'].value()
            control['slider'].setSliderPosition(int(value))
            eventlet.spawn(self.controller.set_single_pose, servo_id, value)

    def handle_capture_current(self):
        with self.servo_update:
            results = self.controller.get_pose(range(len(self.servo_controls)))
            for ident, angle in results.iteritems():
                control = self.servo_controls[ident]
                control['slider'].setSliderPosition(int(angle))
                control['doublespin'].setValue(angle)

    def read_settings(self, config):
        if not config.has_section('servo'):
            return

        self.ui.typeCombo.setCurrentIndex(config.getint('servo', 'type'))
        self.ui.serialPortCombo.setEditText(config.get('servo', 'port'))
        self.ui.modelEdit.setText(config.get('servo', 'model'))
        self.ui.servoCountSpin.setValue(config.getint('servo', 'count'))

    def write_settings(self, config):
        config.add_section('servo')

        config.set('servo', 'type', self.ui.typeCombo.currentIndex())
        config.set('servo', 'port', self.ui.serialPortCombo.currentText())
        config.set('servo', 'model', self.ui.modelEdit.text())
        config.set('servo', 'count', self.ui.servoCountSpin.value())

class Mtool(QtGui.QMainWindow):
    CONFIG_FILE = os.path.expanduser('~/.config/mtool/mtool.ini')

    def __init__(self, parent=None):
        super(Mtool, self).__init__(parent)

        self.ui = mtool_main_window.Ui_MtoolMainWindow()
        self.ui.setupUi(self)

        self.servo_tab = ServoTab(self.ui)

        self.read_settings()

    def closeEvent(self, event):
        self.write_settings()
        event.accept()

    def read_settings(self):
        config = ConfigParser.ConfigParser()
        config.read(self.CONFIG_FILE)

        self.servo_tab.read_settings(config)

    def write_settings(self):
        config = ConfigParser.ConfigParser()

        self.servo_tab.write_settings(config)

        config_dir = os.path.dirname(self.CONFIG_FILE)
        if not os.path.exists(config_dir):
            os.mkdir(config_dir)
        config.write(open(self.CONFIG_FILE, 'w'))

def eventlet_pyside_mainloop(app):
    timer = QtCore.QTimer()
    def on_timeout():
        eventlet.sleep()
    timer.timeout.connect(on_timeout)
    timer.start(10)
    app.exec_()

def main():
    app = QtGui.QApplication(sys.argv)
    app.setApplicationName('mtool')

    mtool = Mtool()
    mtool.show()

    eventlet_pyside_mainloop(app)
    sys.exit()

if __name__ == '__main__':
    main()
