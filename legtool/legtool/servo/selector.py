#!/usr/bin/python

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

import trollius as asyncio
from trollius import From, Return
import math
import pygazebo
from pygazebo.msg import joint_cmd_pb2

from . import herkulex

# Constants used in the 'enable_power' method.
(# No power is applied, servos can be manipulated
 POWER_FREE,
 # Resistance is applied, but not actively controlled
 POWER_BRAKE,
 # Power is used to maintain position
 POWER_ENABLE) = range(3)

class HerkuleXController(object):

    def __init__(self, serial_port=None):
        if serial_port is None:
            raise RuntimeError('serial port required')

        self.port = herkulex.HerkuleX(serial_port=serial_port)
        self.default_pose_time = 0.03

    @asyncio.coroutine
    def start(self):
        yield From(self.port.start())
        raise Return(self)

    @asyncio.coroutine
    def flush(self):
        yield From(self.port.flush())

    @asyncio.coroutine
    def reboot(self):
        yield From(self.port.reboot(self.port.BROADCAST))

    def _angle_to_counts(self, angle_deg):
        return min(1023, max(0, int(512 + angle_deg / 0.325)))

    def _counts_to_angle(self, counts):
        return (counts - 512) * 0.325

    @asyncio.coroutine
    def set_pose(self, id_to_deg_map, pose_time=None):
        if pose_time is None:
            pose_time = self.default_pose_time

        yield From(
            self.port.s_jog(
                time_ms=pose_time * 1000,
                targets=[(ident, self._angle_to_counts(angle), 0)
                         for ident, angle in id_to_deg_map.iteritems()]))

    @asyncio.coroutine
    def set_single_pose(self, ident, angle_deg, pose_time=None):
        yield From(self.set_pose({ident: angle_deg}, pose_time))

    @asyncio.coroutine
    def enable_power(self, state, idents=None):
        """Enable the power state of one or more servos.

        :param value: one of POWER_FREE, POWER_BRAKE, or POWER_ENABLE
        :param idents: optional list of identifiers to configure, if
            None, all servos are configured
        """
        value = 0
        if state == POWER_FREE:
            value = 0x00
        elif state == POWER_BRAKE:
            value = 0x40
        elif state == POWER_ENABLE:
            value = 0x60

        if idents is None:
            idents = [self.port.BROADCAST]

        for ident in idents:
            yield From(
                self.port.ram_write(
                    ident, self.port.REG_TORQUE_CONTROL, [value]))

    @asyncio.coroutine
    def configure_servo(self, settings, idents):
        """Configure servoes using a dict of settings"""
        for key, val in sorted(settings.items()):
            if key == 'dead_zone':
                addr = self.port.REG_DEAD_ZONE
                data = [int(val)]
                assert 0 <= data[0] <= 254
            else:
                raise Exception('Unknown configuration key %r' % key)

            for ident in idents:
                yield From(self.port.ram_write(ident, addr, data))


    @asyncio.coroutine
    def get_pose(self, idents=[]):
        """Determine the current pose of the requested servos.

        :returns: a dictionary mapping identifier to angle in degrees
        """
        result = {}

        for ident in idents:
            counts = yield From(self.port.position(ident))
            if counts is None:
                continue
            result[ident] = self._counts_to_angle(counts)

        raise Return(result)

    @asyncio.coroutine
    def get_torque(self, idents=[]):
        """Determine the current torque applied by each servo.

        :returns: a dictionary mapping identifier to torque in N*m
        """
        result = {}

        for ident in idents:
            pwm = yield From(self.port.pwm(ident))
            result[ident] = self._pwm_to_torque(pwm)

        raise Return(result)

    @asyncio.coroutine
    def get_temperature(self, idents=[]):
        """Determine the temperature as measured at each servo.

        :returns: a dictionary mapping identifier to temperature in C"""
        result = {}
        for ident in idents:
            result[ident] = yield From(self.port.temperature_C(ident))
        raise Return(result)

    @asyncio.coroutine
    def get_voltage(self, idents=[]):
        """Determine the voltage as measured at each servo.

        :returns: a dictionary mapping identifier to voltage"""
        result = {}
        for ident in idents:
            result[ident] = yield From(self.port.voltage(ident))
        raise Return(result)

    @asyncio.coroutine
    def get_clear_status(self, ident):
        """Get a list of pending servo errors, and clear these errors,
        if any.
        @returns list of strings -- status flags
        """
        try:
            status = yield From(self.port.status(ident))
        except asyncio.TimeoutError:
            status = None

        if status is None:
            raise Return(['offline'])

        if status.has_errors():
            yield From(self.port.clear_errors(ident))
        raise Return(status.active_flags())

    @asyncio.coroutine
    def set_leds(self, ident, red=False, green=False, blue=False):
        flags = 0
        if red:
            flags |= self.port.LED_RED
        if green:
            flags |= self.port.LED_GREEN
        if blue:
            flags |= self.port.LED_BLUE

        yield From(self.port.set_leds(ident, flags))

    _MJMECH_ID = 99

    @asyncio.coroutine
    def mjmech_fire(self, fire_time, fire_pwm):
        fire_time_ticks = int(fire_time / 0.10)
        assert 0 <= fire_time_ticks <= 255, (
            'Invalid fire_time value: %r sec (%d ticks)' % (
                fire_time, fire_time_ticks))

        fire_pwm_raw = int(fire_pwm * 255.0)
        assert 0 <= fire_pwm_raw <= 255, (
            'Invalid fire_pwm value: %r (%d raw)' % (
                fire_pwm, fire_pwm_raw))
        yield From(self.port.ram_write(
                self._MJMECH_ID, 80, [fire_time_ticks, fire_pwm_raw]))


    @asyncio.coroutine
    def  mjmech_set_outputs(self, laser=False, green=False, blue=False,
                            agitator=0):
        flags = 0
        if laser:
            flags |= self.port.LED_RED
        if green:
            flags |= self.port.LED_GREEN
        if blue:
            flags |= self.port.LED_BLUE
        agitator_raw = int(agitator * 255.0)
        assert 0 <= agitator_raw <= 255, (
            'Invalid agitator value: %r (%d raw)' % (
                agitator, agitator_raw))
        yield From(self.port.ram_write(
                self._MJMECH_ID, 82, [agitator_raw, flags]))



class GazeboController(object):
    def __init__(self, model_name, servo_name_map=None):
        self.model_name = model_name

        self.servo_name_map = dict(
            (key, model_name + '::' + value)
            for key, value in servo_name_map.iteritems())

    @asyncio.coroutine
    def start(self):
        model_name = self.model_name

        self.manager = yield From(pygazebo.connect())

        self.publisher = yield From(self.manager.advertise(
            '/gazebo/default/%s/joint_cmd' % model_name,
            'gazebo.msgs.JointCmd'))

        yield From(self.publisher.wait_for_listener())

        self.subscriber = self.manager.subscribe(
            '/gazebo/default/%s/joint_cmd' % model_name,
            'gazebo.msgs.JointCmd',
            self._receive_joint_cmd)


        self.joint_cmd = joint_cmd_pb2.JointCmd()
        self.joint_cmd.axis = 0
        self.joint_cmd.position.target = 0
        self.joint_cmd.position.p_gain = 10.0
        self.joint_cmd.position.i_gain = 2.0
        self.joint_cmd.position.d_gain = 0.2

        self._servo_angles = {}

    @asyncio.coroutine
    def flush(self):
        raise Return(None)

    @asyncio.coroutine
    def reboot(self):
        pass

    def _receive_joint_cmd(self, data):
        joint_cmd = joint_cmd_pb2.JointCmd.FromString(data)
        index = self.servo_name_map.index(self.joint_cmd.name)
        if index < 0:
            return

        if (not joint_cmd.HasField('position') or
            not joint_cmd.position.HasField('target')):
            return

        self._servo_angles[index] = joint_cmd.position.target

    @asyncio.coroutine
    def set_pose(self, id_to_deg_map, pose_time=None):
        for ident, angle in id_to_deg_map.iteritems():
            self.joint_cmd.name = self.servo_name_map[ident]
            self.joint_cmd.position.target = math.radians(angle)
            yield From(self.publisher.publish(self.joint_cmd))
            self._servo_angles[ident] = angle

    @asyncio.coroutine
    def get_pose(self, idents=[]):
        """Determine the current pose of the requested servos.

        :returns: a dictionary mapping identifier to angle in degrees
        """
        result = {}

        for ident in idents:
            result[ident] = self._servo_angles.get(ident)

        raise Return(result)

    @asyncio.coroutine
    def get_torque(self, idents=[]):
        """Determine the current torque applied by each servo.

        :returns: a dictionary mapping identifier to torque in N*m
        """
        result = {}

        for ident in idents:
            result[ident] = None

        raise Return(result)

    @asyncio.coroutine
    def set_single_pose(self, ident, angle_deg, pose_time=None):
        yield From(self.set_pose({ident: angle_deg}, pose_time))

    @asyncio.coroutine
    def enable_power(self, state, idents=None):
        """Enable the power state of one or more servos.

        :param value: one of POWER_FREE, POWER_BRAKE, or POWER_ENABLE
        :param idents: optional list of identifiers to configure, if
            None, all servos are configured
        """
        raise Return(None)

    @asyncio.coroutine
    def configure_servo(self, settings, idents):
        """Configure servoes using a dict of settings"""
        pass

    @asyncio.coroutine
    def set_leds(self, ident, red=False, green=False, blue=False):
        pass

    @asyncio.coroutine
    def mjmech_fire(self, fire_time, fire_pwm):
        pass

    @asyncio.coroutine
    def mjmech_set_outputs(self, laser=False, green=False, blue=False,
                           agitator=0):
        pass

    @asyncio.coroutine
    def get_clear_status(self, ident):
        raise Return([])

@asyncio.coroutine
def select_servo(servo_type, **kwargs):
    if servo_type == 'herkulex':
        if 'model_name' in kwargs:
            kwargs.pop('model_name')
        if 'servo_name_map' in kwargs:
            kwargs.pop('servo_name_map')
        result = HerkuleXController(**kwargs)
    elif servo_type == 'gazebo':
        if 'serial_port' in kwargs:
            kwargs.pop('serial_port')
        result = GazeboController(**kwargs)
    else:
        raise RuntimeError('unknown servo type: ' + servo_type)
    yield From(result.start())
    raise Return(result)
