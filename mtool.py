#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

# TODO
#  * Implement single leg IK window

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

        self.ui.addPoseButton.clicked.connect(self.handle_add_pose)
        self.ui.removePoseButton.clicked.connect(self.handle_remove_pose)
        self.ui.moveToPoseButton.clicked.connect(
            spawn(self.handle_move_to_pose))
        self.ui.updatePoseButton.clicked.connect(self.handle_update_pose)

        self.controller = None
        self.servo_update = BoolContext()

    def poses(self):
        result = []
        for i in range(self.ui.poseList.count()):
            result.append(self.ui.poseList.item(i).text())
        return result

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

    def add_list_pose(self, name):
        self.ui.poseList.addItem(name)
        item = self.ui.poseList.item(self.ui.poseList.count() - 1)
        item.setFlags(QtCore.Qt.ItemIsEnabled |
                      QtCore.Qt.ItemIsSelectable |
                      QtCore.Qt.ItemIsEditable |
                      QtCore.Qt.ItemIsSelectable)
        return item

    def get_new_pose_name(self):
        poses = set([self.ui.poseList.item(x).text()
                     for x in range(self.ui.poseList.count())])
        count = 0
        while True:
            name = 'new_pose_%d' % count
            if name not in poses:
                return name
            count += 1

    def generate_pose_data(self):
        return dict(
            [ (i, control['doublespin'].value())
              for i, control in enumerate(self.servo_controls) ])

    def handle_add_pose(self):
        pose_name = self.get_new_pose_name()
        item = self.add_list_pose(pose_name)
        item.setData(QtCore.Qt.UserRole, self.generate_pose_data())
        self.ui.poseList.editItem(item)

    def handle_remove_pose(self):
        if self.ui.poseList.currentRow() < 0:
            return
        pose_name = self.ui.poseList.currentItem().text()
        del self.poses[pose_name]
        self.ui.poseList.takeItem(self.ui.poseList.currentRow())

    def handle_move_to_pose(self):
        if self.ui.poseList.currentRow() < 0:
            return

        values = self.ui.poseList.currentItem().data(QtCore.Qt.UserRole)
        self.controller.set_pose(values)
        with self.servo_update:
            for ident, angle_deg in values.iteritems():
                control = self.servo_controls[ident]
                control['slider'].setSliderPosition(int(angle_deg))
                control['doublespin'].setValue(angle_deg)

    def handle_update_pose(self):
        if self.ui.poseList.currentRow() < 0:
            return

        self.ui.poseList.currentItem().setData(
            QtCore.Qt.UserRole, self.generate_pose_data())

    def read_settings(self, config):
        if not config.has_section('servo'):
            return

        self.ui.typeCombo.setCurrentIndex(config.getint('servo', 'type'))
        self.ui.serialPortCombo.setEditText(config.get('servo', 'port'))
        self.ui.modelEdit.setText(config.get('servo', 'model'))
        self.ui.servoCountSpin.setValue(config.getint('servo', 'count'))

        if not config.has_section('servo.poses'):
            return

        for name, value in config.items('servo.poses'):
            this_data = {}
            for element in value.split(','):
                ident, angle_deg = element.split('=')
                this_data[int(ident)] = float(angle_deg)
            item = self.add_list_pose(name)
            item.setData(QtCore.Qt.UserRole, this_data)


    def write_settings(self, config):
        config.add_section('servo')
        config.add_section('servo.poses')

        config.set('servo', 'type', self.ui.typeCombo.currentIndex())
        config.set('servo', 'port', self.ui.serialPortCombo.currentText())
        config.set('servo', 'model', self.ui.modelEdit.text())
        config.set('servo', 'count', self.ui.servoCountSpin.value())

        for row in range(self.ui.poseList.count()):
            item = self.ui.poseList.item(row)
            pose_name = item.text()
            values = item.data(QtCore.Qt.UserRole)
            config.set(
                'servo.poses', pose_name,
                ','.join(['%d=%.2f' % (ident, angle_deg)
                          for ident, angle_deg in values.iteritems()]))

class LegConfig(object):
    present = False
    coxa_ident = 0
    coxa_sign = 1
    femur_ident = 0
    femur_sign = 1
    tibia_ident = 0
    tibia_sign = 1

    def __str__(self):
        return '%s,%d,%d,%d,%d,%d,%d' % (
            self.present,
            self.coxa_ident, self.coxa_sign,
            self.femur_ident, self.femur_sign,
            self.tibia_ident, self.femur_sign)

    @staticmethod
    def from_string(data):
        fields = data.split(',')

        result = LegConfig()

        result.present = True if fields[0].lower() == 'true' else False
        (result.coxa_ident, result.coxa_sign,
         result.femur_ident, result.femur_sign,
         result.tibia_ident, result.tibia_sign) = [int(x) for x in fields[1:]]

        return result


class IkTester(object):
    def __init__(self, graphics_view):
        self.graphics_scene = QtGui.QGraphicsScene()
        self.graphics_view = graphics_view

        self.graphics_view.setTransform(QtGui.QTransform().scale(1, -1))
        self.graphics_view.setScene(self.graphics_scene)

        self.graphics_scene.addLine(-1, 0, 1, 0)
        self.graphics_scene.addLine(0, -1, 0, 1)
        for x in range(-10, 11):
            self.graphics_scene.addLine(-0.02, x / 10., 0.02, x / 10.0)
            self.graphics_scene.addLine(x / 10., -0.02, x / 10.0, 0.02)

        labels = [self.graphics_scene.addText('') for x in range(4)]
        for label in labels:
            label.setFlag(QtGui.QGraphicsItem.ItemIgnoresTransformations)

        (self.label_x_plus,
         self.label_x_minus,
         self.label_y_plus,
         self.label_y_minus) = labels

        # Apparently it is really hard to get QGraphicsTextItems to be
        # aligned anything other than left|upper.  Thus, just hack in
        # some offsets which kind of work on my monitor for now.
        self.label_x_plus.setPlainText('+10mm')
        self.label_x_plus.setPos(0.7, 0)

        self.label_x_minus.setPlainText('-10mm')
        self.label_x_minus.setPos(-1, 0)

        self.label_y_plus.setPlainText('+10mm')
        self.label_y_plus.setPos(0, 1)

        self.label_y_minus.setPlainText('-10mm')
        self.label_y_minus.setPos(0, -.9)

    def fit_in_view(self):
        self.graphics_view.fitInView(QtCore.QRectF(-1, -1, 2, 2))

    def set_config(self, coxa_servo, coxa_sign,
                   femur_servo, femur_sign,
                   tibia_servo, tibia_sign,
                   idle_values, minimum_values, maximum_values,
                   x_offset_mm, y_offset_mm, z_offset_mm,
                   coxa_length_mm, femur_length_mm, tibia_length_mm,
                   plane):
        self.coxa_servo = coxa_servo
        self.coxa_sign = coxa_sign
        self.femur_servo = femur_servo
        self.femur_sign = femur_sign
        self.tibia_servo = tibia_servo
        self.tibia_sign = tibia_sign

        self.idle_values = idle_values
        self.minimum_values = minimum_values
        self.maximum_values = maximum_values
        self.x_offset_mm = x_offset_mm
        self.y_offset_mm = y_offset_mm
        self.z_offset_mm = z_offset_mm
        self.coxa_length_mm = coxa_length_mm
        self.femur_length_mm = femur_length_mm
        self.tibia_length_mm = tibia_length_mm
        self.plane = plane

        self.update_scene()

    def update_scene(self):
        pass



class IkConfigTab(object):
    def __init__(self, ui, servo_tab):
        self.ui = ui
        self.servo_tab = servo_tab
        self.legs = {}
        self.in_number_changed = BoolContext()

        self.ik_tester = IkTester(self.ui.ikTestView)

        self.ui.tabWidget.currentChanged.connect(self.handle_current_changed)
        self.handle_current_changed()

        self.ui.legSpin.valueChanged.connect(self.handle_leg_number_changed)
        self.handle_leg_number_changed()

        for combo in [self.ui.legPresentCombo,
                      self.ui.coxaSignCombo,
                      self.ui.femurSignCombo,
                      self.ui.tibiaSignCombo]:
            combo.currentIndexChanged.connect(self.handle_leg_data_change)

        for spin in [self.ui.coxaServoSpin,
                     self.ui.femurServoSpin,
                     self.ui.tibiaServoSpin]:
            spin.valueChanged.connect(self.handle_leg_data_change)

        for combo in [self.ui.idleCombo,
                      self.ui.minimumCombo,
                      self.ui.maximumCombo,
                      self.ui.planeCombo]:
            combo.currentIndexChanged.connect(self.handle_ik_config_change)

        for spin in [self.ui.xOffsetSpin,
                     self.ui.yOffsetSpin,
                     self.ui.zOffsetSpin,
                     self.ui.coxaLengthSpin,
                     self.ui.femurLengthSpin,
                     self.ui.tibiaLengthSpin]:
            spin.valueChanged.connect(self.handle_ik_config_change)

    def handle_current_changed(self, index=1):
        if index != 1:
            return

        # Our tab is now current.  Make sure any links to the servo
        # tab are updated properly.
        poses = self.servo_tab.poses()
        combos = [self.ui.idleCombo,
                  self.ui.minimumCombo,
                  self.ui.maximumCombo]
        for pose in poses:
            for combo in combos:
                if combo.findText(pose) < 0:
                    combo.addItem(pose)

        self.ik_tester.fit_in_view()

    def update_config_enable(self):
        enable = self.ui.legPresentCombo.currentIndex() == 0

        for x in [self.ui.coxaServoSpin, self.ui.coxaSignCombo,
                  self.ui.femurServoSpin, self.ui.femurSignCombo,
                  self.ui.tibiaServoSpin, self.ui.tibiaSignCombo]:
            x.setEnabled(enable)

    def handle_leg_data_change(self):
        if self.in_number_changed.value:
            return

        leg_num = self.ui.legSpin.value()
        leg_config = self.legs.get(leg_num, LegConfig())

        leg_config.present = (True
                              if self.ui.legPresentCombo.currentIndex() == 0
                              else False)

        def combo_sign(combo):
            return 1 if combo.currentIndex() == 0 else -1

        leg_config.coxa_ident = self.ui.coxaServoSpin.value()
        leg_config.coxa_sign = combo_sign(self.ui.coxaSignCombo)
        leg_config.femur_ident = self.ui.femurServoSpin.value()
        leg_config.femur_sign = combo_sign(self.ui.femurSignCombo)
        leg_config.tibia_ident = self.ui.tibiaServoSpin.value()
        leg_config.tibia_sign = combo_sign(self.ui.tibiaSignCombo)

        self.update_config_enable()

        self.legs[leg_num] = leg_config


    def handle_leg_number_changed(self):
        with self.in_number_changed:
            leg_config = self.legs.get(self.ui.legSpin.value(), LegConfig())

            self.ui.legPresentCombo.setCurrentIndex(
                0 if leg_config.present else 1)

            self.ui.coxaServoSpin.setValue(leg_config.coxa_ident)
            self.ui.coxaSignCombo.setCurrentIndex(
                0 if leg_config.coxa_sign > 0 else 1)

            self.ui.femurServoSpin.setValue(leg_config.femur_ident)
            self.ui.femurSignCombo.setCurrentIndex(
                0 if leg_config.femur_sign > 0 else 1)

            self.ui.tibiaServoSpin.setValue(leg_config.tibia_ident)
            self.ui.tibiaSignCombo.setCurrentIndex(
                0 if leg_config.tibia_sign > 0 else 1)

            self.update_config_enable()

    def handle_ik_config_change(self):
        # Update the visualization and the IK solver with our new
        # configuration.
        pass

    def get_float_configs(self):
        return [(self.ui.xOffsetSpin, 'x_offset'),
                (self.ui.yOffsetSpin, 'y_offset'),
                (self.ui.zOffsetSpin, 'z_offset'),
                (self.ui.coxaLengthSpin, 'coxa_length'),
                (self.ui.femurLengthSpin, 'femur_length'),
                (self.ui.tibiaLengthSpin, 'tibia_length')]

    def read_settings(self, config):
        self.handle_current_changed()

        if config.has_section('ikconfig'):
            def set_combo(combo, name):
                value = config.get('ikconfig', name)
                for i in range(combo.count()):
                    if combo.itemText(i) == value:
                        combo.setCurrentIndex(i)
                        break

            set_combo(self.ui.idleCombo, 'idle_pose')
            set_combo(self.ui.minimumCombo, 'minimum_pose')
            set_combo(self.ui.maximumCombo, 'maximum_pose')

            for spin, name in self.get_float_configs():
                spin.setValue(config.getfloat('ikconfig', name))

        if config.has_section('ikconfig.legs'):
            for leg_name, value in config.items('ikconfig.legs'):
                leg_num = int(leg_name.split('.')[1])
                self.legs[leg_num] = LegConfig.from_string(value)

        for i in range(6):
            if i not in self.legs:
                self.legs[i] = LegConfig()

        self.handle_leg_number_changed()

    def write_settings(self, config):
        config.add_section('ikconfig')

        config.set('ikconfig', 'idle_pose', self.ui.idleCombo.currentText())
        config.set('ikconfig', 'minimum_pose',
                   self.ui.minimumCombo.currentText())
        config.set('ikconfig', 'maximum_pose',
                   self.ui.maximumCombo.currentText())

        for spin, name in self.get_float_configs():
            config.set('ikconfig', name, spin.value())

        config.add_section('ikconfig.legs')
        for leg_num, leg_config in self.legs.iteritems():
            config.set('ikconfig.legs', 'leg.%d' % leg_num, str(leg_config))


class Mtool(QtGui.QMainWindow):
    CONFIG_FILE = os.path.expanduser('~/.config/mtool/mtool.ini')

    def __init__(self, parent=None):
        super(Mtool, self).__init__(parent)

        self.ui = mtool_main_window.Ui_MtoolMainWindow()
        self.ui.setupUi(self)

        self.servo_tab = ServoTab(self.ui)
        self.ikconfig_tab = IkConfigTab(self.ui, self.servo_tab)

        self.read_settings()

    def closeEvent(self, event):
        self.write_settings()
        event.accept()

    def read_settings(self):
        config = ConfigParser.ConfigParser()
        config.read(self.CONFIG_FILE)

        self.servo_tab.read_settings(config)
        self.ikconfig_tab.read_settings(config)

    def write_settings(self):
        config = ConfigParser.ConfigParser()

        self.servo_tab.write_settings(config)
        self.ikconfig_tab.write_settings(config)

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
