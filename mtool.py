#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

# TODO
#  * Populate servo controls
#  * Save and restore settings in persistent way
#  * Connect up servo controls to live devices
#  * Implement saving and restoring poses

import eventlet
import functools
import sys

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

import mtool_main_window
import servo_controller

pool = None

def spawn(callback):
    def task():
        eventlet.spawn(callback)

    return task

class Mtool(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(Mtool, self).__init__(parent)

        self.servo_controls = []

        self.ui = mtool_main_window.Ui_MtoolMainWindow()
        self.ui.setupUi(self)

        self.ui.statusText.setText('not connected')
        self.ui.connectButton.clicked.connect(
            spawn(self.handle_connect_clicked))

        servo_layout = QtGui.QVBoxLayout()
        servo_layout.setSpacing(0)
        servo_layout.setContentsMargins(0, 0, 0, 0)
        self.ui.scrollContents.setLayout(servo_layout)


        self.ui.servoCountSpin.valueChanged.connect(self.handle_servo_count)
        self.handle_servo_count()


        self.update_connected(False)

        self.controller = None

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
                spawn(functools.partial(self.handle_servo_slider, servo_id)))
            doublespin.valueChanged.connect(
                spawn(functools.partial(self.handle_servo_spin, servo_id)))

            self.ui.scrollContents.layout().addWidget(widget)

            self.servo_controls.append({
                    'widget': widget,
                    'label': label,
                    'slider': slider,
                    'doublespin': doublespin})


    def update_connected(self, value):
        self.ui.controlGroup.setEnabled(value)
        self.ui.posesGroup.setEnabled(value)

    def handle_servo_slider(self, servo_id):
        control = self.servo_controls[servo_id]
        value = control['slider'].value()
        control['doublespin'].setValue(value)
        self.controller.set_single_pose(servo_id, value)

    def handle_servo_spin(self, servo_id):
        control = self.servo_controls[servo_id]
        value = control['doublespin'].value()
        control['slider'].setSliderPosition(int(value))
        self.controller.set_single_pose(servo_id, value)

def eventlet_pyside_mainloop(app):
    timer = QtCore.QTimer()
    def on_timeout():
        eventlet.sleep()
    timer.timeout.connect(on_timeout)
    timer.start(50)
    app.exec_()

def main():
    app = QtGui.QApplication(sys.argv)

    mtool = Mtool()
    mtool.show()

    eventlet_pyside_mainloop(app)
    sys.exit()

if __name__ == '__main__':
    main()
