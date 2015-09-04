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

import os
import sys

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

from trollius import Task

sys.path.append(
    os.path.join(os.path.dirname(
            os.path.realpath(__file__)), '../../src/build'))
import _legtool
from _legtool import Point3D

from .common import BoolContext
from . import settings
from . import graphics_scene

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
    (PLANE_XY,
     PLANE_XZ,
     PLANE_YZ) = range(3)

    def __init__(self, servo_tab, ikconfig_tab, graphics_view):
        self.servo_tab = servo_tab
        self.ikconfig_tab = ikconfig_tab
        self.graphics_scene = graphics_scene.GraphicsScene()
        self.graphics_scene.sceneMouseMoveEvent.connect(
            self.handle_mouse_move)
        self.graphics_scene.sceneMousePressEvent.connect(
            self.handle_mouse_press)
        self.graphics_scene.sceneMouseReleaseEvent.connect(
            self.handle_mouse_release)
        self.graphics_view = graphics_view

        self.graphics_view.setTransform(QtGui.QTransform().scale(1, -1))
        self.graphics_view.setScene(self.graphics_scene)

        self.usable_rects = {}
        self.grid_count = 20
        for x in range(-self.grid_count, self.grid_count + 1):
            for y in range(-self.grid_count, self.grid_count + 1):
                self.usable_rects[(x, y)] = \
                    self.graphics_scene.addRect(
                    (x - 0.5) / self.grid_count,
                    (y - 0.5) / self.grid_count,
                    1.0 / self.grid_count, 1.0 / self.grid_count)

        for rect in self.usable_rects.itervalues():
            rect.setPen(QtGui.QPen(QtCore.Qt.NoPen))
            rect.setZValue(-20)

        self.axes = graphics_scene.AxesItem()
        self.graphics_scene.addItem(self.axes)
        self.axes.x_suffix = 'mm'
        self.axes.y_suffix = 'mm'

        self.length_scale = 50
        self.axes.x_scale = self.axes.y_scale = self.length_scale
        self.axes.update()

        self.ik_config = None

    def resize(self):
        self.fit_in_view()

    def fit_in_view(self):
        self.graphics_view.fitInView(QtCore.QRectF(-1, -1, 2, 2))

    def set_config(self,
                   leg_number,
                   x_offset_mm, y_offset_mm, z_offset_mm,
                   leg_ik,
                   plane,
                   length_scale,
                   speed_scale,
                   speed_axis):
        self.leg_number = leg_number
        self.x_offset_mm = x_offset_mm
        self.y_offset_mm = y_offset_mm
        self.z_offset_mm = z_offset_mm
        self.leg_ik = leg_ik
        self.plane = plane
        self.length_scale = length_scale
        self.speed_scale = speed_scale
        self.speed_axis = speed_axis

        self.axes.x_scale = self.axes.y_scale = self.length_scale
        self.axes.update()
        self.update_scene()

    def coord_to_point(self, coord):
        coord1 = coord[0] * self.length_scale
        coord2 = coord[1] * self.length_scale

        point_mm = Point3D(self.x_offset_mm,
                           self.y_offset_mm,
                           self.z_offset_mm)
        if self.plane == self.PLANE_XY:
            point_mm += Point3D(coord1, coord2, 0.0)
        elif self.plane == self.PLANE_XZ:
            point_mm += Point3D(coord1, 0.0, coord2)
        elif self.plane == self.PLANE_YZ:
            point_mm += Point3D(0.0, coord1, coord2)
        else:
            raise RuntimeError('invalid plane:' + str(self.plane))

        return point_mm

    def find_worst_case_speed_mm_s(self, ik, point_mm, direction_mm):
        step_mm = 0.01

        nominal = ik.do_ik(point_mm)
        if not nominal.valid():
            return None

        result = None
        normalized = direction_mm.scaled(1.0 / direction_mm.length())
        point_step_mm = direction_mm.scaled(step_mm)

        advanced = ik.do_ik(point_mm + point_step_mm)

        for (a, b) in zip(nominal.joints, advanced.joints):
            assert(a.ident == b.ident)

            if a.angle_deg == b.angle_deg:
                continue

            this_speed_mm_s = (ik.config().servo_speed_dps * step_mm /
                               abs(a.angle_deg - b.angle_deg))
            if result is None or this_speed_mm_s < result:
                result = this_speed_mm_s
        return result

    def update_scene(self):
        ik = self.leg_ik

        axis = self.speed_axis

        for (x, y), rect in self.usable_rects.iteritems():
            point_mm = self.coord_to_point((float(x) / self.grid_count,
                                            float(y) / self.grid_count))
            result = ik.do_ik(point_mm)
            if not result.valid():
                rect.setBrush(QtGui.QBrush(QtCore.Qt.red))
            else:
                speed = self.find_worst_case_speed_mm_s(
                    ik, point_mm, axis)
                if speed is None:
                    rect.setBrush(QtGui.QBrush())
                else:
                    val = int(min(255, 255 * speed / self.speed_scale))
                    color = QtGui.QColor(255 - val, 255, 255 - val)
                    rect.setBrush(QtGui.QBrush(color))

    def handle_mouse_press(self, cursor):
        Task(self.servo_tab.set_power('drive'))

        self.handle_mouse_move(cursor)

    def handle_mouse_release(self):
        Task(self.servo_tab.set_power('brake'))

    def handle_mouse_move(self, cursor):
        point_mm = self.coord_to_point((cursor.x(), cursor.y()))

        self.servo_tab.ui.resultsEdit.setText('')

        result = self.leg_ik.do_ik(point_mm)
        if not result.valid():
            # This option isn't possible
            return

        message = ''

        command = {}
        for joint in result.joints:
            command[joint.ident] = joint.angle_deg
            message += '%d: %f\n' % (joint.ident, joint.angle_deg)

        if hasattr(self.leg_ik, 'forward'):
            forward = self.leg_ik.forward(result)

            message += '\nshoulder: %.1f,%.1f,%.1f\n' % (
                forward.shoulder.x, forward.shoulder.y, forward.shoulder.z)
            message += 'femur: %.1f,%.1f,%.1f\n' % (
                forward.femur.x, forward.femur.y, forward.femur.z)
            message += 'tibia: %.1f,%.1f,%.1f\n' % (
                forward.tibia.x, forward.tibia.y, forward.tibia.z)
        self.servo_tab.ui.resultsEdit.setText(message)

        Task(self.servo_tab.set_pose(command))


class IkConfigTab(object):
    def __init__(self, ui, servo_tab):
        self.ui = ui
        self.servo_tab = servo_tab
        self.legs = {}
        self.in_number_changed = BoolContext()

        self.ik_tester = IkTester(servo_tab, self, self.ui.ikTestView)

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

        self.ui.iktypeCombo.currentIndexChanged.connect(
            self.handle_iktype_change)

        for combo in [self.ui.idleCombo,
                      self.ui.minimumCombo,
                      self.ui.maximumCombo,
                      self.ui.planeCombo,
                      self.ui.speedAxisCombo]:
            combo.currentIndexChanged.connect(self.handle_ik_config_change)

        for spin in [self.ui.xOffsetSpin,
                     self.ui.yOffsetSpin,
                     self.ui.zOffsetSpin,
                     self.ui.coxaLengthSpin,
                     self.ui.femurLengthSpin,
                     self.ui.tibiaLengthSpin,
                     self.ui.lengthScaleSpin,
                     self.ui.speedScaleSpin,
                     self.ui.mammalFemurOutSpin,
                     self.ui.mammalFemurCwSpin,
                     self.ui.mammalFemurUpSpin,
                     self.ui.mammalFemurLengthSpin,
                     self.ui.mammalTibiaLengthSpin]:
            spin.valueChanged.connect(self.handle_ik_config_change)

        self.handle_ik_config_change()

    def resizeEvent(self, event):
        self.ik_tester.resize()

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

    def get_all_legs(self):
        '''Return a list of all available leg numbers.'''
        return range(0, self.ui.legSpin.maximum() + 1)

    def get_enabled_legs(self):
        '''Return a list of all leg numbers which are currently
        enabled.'''
        return [x for x in self.get_all_legs()
                if self.legs.get(x, LegConfig()).present]

    def get_leg_ik(self, leg_number):
        '''Return an IK solver for the given leg number.'''
        idle_values = self.servo_tab.pose(
            self.ui.idleCombo.currentText())
        minimum_values = self.servo_tab.pose(
            self.ui.minimumCombo.currentText())
        maximum_values = self.servo_tab.pose(
            self.ui.maximumCombo.currentText())

        leg = self.legs.get(leg_number, LegConfig())

        def set_joints(joint1, ident1, joint2, ident2, joint3, ident3):
            joint1.min_deg = minimum_values[ident1]
            joint1.idle_deg = idle_values[ident1]
            joint1.max_deg = maximum_values[ident1]
            #result.coxa_mass_kg = self.ui.coxaMassSpin.value()
            joint1.ident = ident1

            joint2.min_deg = minimum_values[ident2]
            joint2.idle_deg = idle_values[ident2]
            joint2.max_deg = maximum_values[ident2]
            #result.femur_mass_kg = self.ui.femurMassSpin.value()
            joint2.ident = ident2

            joint3.min_deg = minimum_values[ident3]
            joint3.idle_deg = idle_values[ident3]
            joint3.max_deg = maximum_values[ident3]
            #result.tibia_mass_kg = self.ui.tibiaMassSpin.value()
            joint3.ident = ident3

        if self.ui.iktypeCombo.currentIndex() == 0:
            # Lizard.
            result = _legtool.LizardIKConfig()

            set_joints(result.coxa, leg.coxa_ident,
                       result.femur, leg.femur_ident,
                       result.tibia, leg.tibia_ident)

            result.coxa.length_mm = self.ui.coxaLengthSpin.value()
            result.coxa.sign = leg.coxa_sign

            result.femur.length_mm = self.ui.femurLengthSpin.value()
            result.femur.sign = leg.femur_sign

            result.tibia.length_mm = self.ui.tibiaLengthSpin.value()
            result.tibia.sign = leg.tibia_sign

            return _legtool.LizardIK(result)

        elif self.ui.iktypeCombo.currentIndex() == 1:
            # Mammal.
            result = _legtool.MammalIKConfig()

            set_joints(result.shoulder, leg.coxa_ident,
                       result.femur, leg.femur_ident,
                       result.tibia, leg.tibia_ident)

            result.femur_attachment_mm.y = self.ui.mammalFemurOutSpin.value()
            result.femur_attachment_mm.x = self.ui.mammalFemurCwSpin.value()
            result.femur_attachment_mm.z = self.ui.mammalFemurUpSpin.value()

            result.shoulder.sign = leg.coxa_sign

            result.femur.length_mm = self.ui.mammalFemurLengthSpin.value()
            result.femur.sign = leg.femur_sign

            result.tibia.length_mm = self.ui.mammalTibiaLengthSpin.value()
            result.tibia.sign = leg.tibia_sign

            return _legtool.MammalIK(result)

    def update_config_enable(self):
        enable = self.ui.legPresentCombo.currentIndex() == 0

        for x in [self.ui.coxaServoSpin, self.ui.coxaSignCombo,
                  self.ui.femurServoSpin, self.ui.femurSignCombo,
                  self.ui.tibiaServoSpin, self.ui.tibiaSignCombo]:
            x.setEnabled(enable)

    def combo_sign(self, combo):
        return 1 if combo.currentIndex() == 0 else -1

    def handle_leg_data_change(self):
        if self.in_number_changed.value:
            return

        leg_num = self.ui.legSpin.value()
        leg_config = self.legs.get(leg_num, LegConfig())

        leg_config.present = (True
                              if self.ui.legPresentCombo.currentIndex() == 0
                              else False)

        leg_config.coxa_ident = self.ui.coxaServoSpin.value()
        leg_config.coxa_sign = self.combo_sign(self.ui.coxaSignCombo)
        leg_config.femur_ident = self.ui.femurServoSpin.value()
        leg_config.femur_sign = self.combo_sign(self.ui.femurSignCombo)
        leg_config.tibia_ident = self.ui.tibiaServoSpin.value()
        leg_config.tibia_sign = self.combo_sign(self.ui.tibiaSignCombo)

        self.update_config_enable()

        self.legs[leg_num] = leg_config

        self.handle_ik_config_change()


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

            self.handle_ik_config_change()

    def get_plane(self):
        value = self.ui.planeCombo.currentIndex()
        if value == 0:
            result = IkTester.PLANE_XY
        elif value == 1:
            result = IkTester.PLANE_XZ
        elif value == 2:
            result = IkTester.PLANE_YZ
        else:
            raise RuntimeError('invalid plane:' + str(value))
        return result

    def handle_iktype_change(self):
        self.ui.geometryStack.setCurrentIndex(
            self.ui.iktypeCombo.currentIndex())
        self.handle_ik_config_change()

    def handle_ik_config_change(self):
        # Update the visualization and the IK solver with our new
        # configuration.
        leg_ik = self.get_leg_ik(self.ui.legSpin.value())

        self.ik_tester.set_config(
            leg_number=self.ui.legSpin.value(),
            x_offset_mm=self.ui.xOffsetSpin.value(),
            y_offset_mm=self.ui.yOffsetSpin.value(),
            z_offset_mm=self.ui.zOffsetSpin.value(),
            leg_ik=leg_ik,
            plane=self.get_plane(),
            length_scale=self.ui.lengthScaleSpin.value(),
            speed_scale=self.ui.speedScaleSpin.value(),
            speed_axis=self._speed_axis(),
            )

    def _speed_axis(self):
        value = self.ui.speedAxisCombo.currentIndex()
        if value == 0:
            return Point3D(1., 0., 0.)
        elif value == 1:
            return Point3D(0., 1., 0.)
        elif value == 2:
            return Point3D(0., 0., 1.)
        elif value == 3:
            return None
        else:
            raise NotImplementedError()

    def get_float_configs(self):
        return [(self.ui.xOffsetSpin, 'x_offset'),
                (self.ui.yOffsetSpin, 'y_offset'),
                (self.ui.zOffsetSpin, 'z_offset'),
                (self.ui.coxaLengthSpin, 'coxa_length'),
                (self.ui.femurLengthSpin, 'femur_length'),
                (self.ui.tibiaLengthSpin, 'tibia_length'),
                (self.ui.coxaMassSpin, 'coxa_mass'),
                (self.ui.femurMassSpin, 'femur_mass'),
                (self.ui.tibiaMassSpin, 'tibia_mass'),
                (self.ui.mammalFemurOutSpin, 'mammal_femur_out'),
                (self.ui.mammalFemurCwSpin, 'mammal_femur_cw'),
                (self.ui.mammalFemurUpSpin, 'mammal_femur_up'),
                (self.ui.mammalFemurLengthSpin, 'mammal_femur_length'),
                (self.ui.mammalTibiaLengthSpin, 'mammal_tibia_length'),
                ]

    def read_settings(self, config):
        self.handle_current_changed()

        if 'ikconfig' not in config:
            return

        ikconfig = config['ikconfig']

        def set_combo(combo, name):
            settings.restore_combo(config, 'ikconfig', combo, name)

        set_combo(self.ui.idleCombo, 'idle_pose')
        set_combo(self.ui.minimumCombo, 'minimum_pose')
        set_combo(self.ui.maximumCombo, 'maximum_pose')
        set_combo(self.ui.iktypeCombo, 'iktype')

        for spin, name in self.get_float_configs():
            if name in ikconfig:
                spin.setValue(ikconfig[name])

        if 'plane' in ikconfig:
            self.ui.planeCombo.setCurrentIndex(ikconfig['plane'])
        if 'scale' in ikconfig:
            self.ui.lengthScaleSpin.setValue(ikconfig['scale'])
        if 'speed_scale' in ikconfig:
            self.ui.speedScaleSpin.setValue(ikconfig['speed_scale'])
        if 'speed_axis' in ikconfig:
            self.ui.speedAxisCombo.setCurrentIndex(ikconfig['speed_axis'])

        if 'legs' in ikconfig:
            legs = ikconfig['legs']

            for leg_name, value in legs.iteritems():
                leg_num = int(leg_name.split('.')[1])
                self.legs[leg_num] = LegConfig.from_string(value)

        for i in range(6):
            if i not in self.legs:
                self.legs[i] = LegConfig()

        self.handle_leg_number_changed()
        self.handle_ik_config_change()

    def write_settings(self, config):
        ikconfig = config.setdefault('ikconfig', {})

        ikconfig['idle_pose'] =  self.ui.idleCombo.currentText()
        ikconfig['minimum_pose'] = self.ui.minimumCombo.currentText()
        ikconfig['maximum_pose'] = self.ui.maximumCombo.currentText()
        ikconfig['iktype'] = self.ui.iktypeCombo.currentText()

        for spin, name in self.get_float_configs():
            ikconfig[name] = spin.value()

        ikconfig['plane'] = self.ui.planeCombo.currentIndex()
        ikconfig['scale'] = self.ui.lengthScaleSpin.value()
        ikconfig['speed_scale'] = self.ui.speedScaleSpin.value()
        ikconfig['speed_axis'] = self.ui.speedAxisCombo.currentIndex()

        legs = ikconfig.setdefault('legs', {})

        for leg_num, leg_config in self.legs.iteritems():
            legs['leg.%d' % leg_num] = str(leg_config)

            if leg_config.present:
                leg = ikconfig.setdefault('leg', {})
                this_leg = leg.setdefault(leg_num, {})
                self.get_leg_ik(leg_num).config().write_settings(this_leg)
