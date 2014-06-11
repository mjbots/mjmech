#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import trollius as asyncio
from trollius import From, Return
import math
import pygazebo
from pygazebo.msg import joint_cmd_pb2

import herkulex

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

    def start(self):
        raise Return(self)

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


class GazeboController(object):
    def __init__(self, model_name):
        self.model_name = model_name

    @asyncio.coroutine
    def start(self):
        model_name = self.model_name

        self.manager = yield From(pygazebo.connect())

        self.publisher = yield From(self.manager.advertise(
            '/gazebo/default/%s/joint_cmd' % model_name,
            'gazebo.msgs.JointCmd'))

        self.subscriber = self.manager.subscribe(
            '/gazebo/default/%s/joint_cmd' % model_name,
            'gazebo.msgs.JointCmd',
            self._receive_joint_cmd)

        self._servo_names = reduce(
            lambda a, b: a + b,
            [ [ '%s::coxa_hinge%d' % (model_name, x),
                '%s::femur_hinge%d' % (model_name, x),
                '%s::tibia_hinge%d' % (model_name, x) ]
              for x in range(4) ] + [ [
                '%s::pan_aeg_hinge' % model_name,
                '%s::weapon_assembly_hinge' % model_name ] ])

        self.joint_cmd = joint_cmd_pb2.JointCmd()
        self.joint_cmd.axis = 0
        self.joint_cmd.position.target = 0
        self.joint_cmd.position.p_gain = 10.0
        self.joint_cmd.position.i_gain = 2.0
        self.joint_cmd.position.d_gain = 0.2

        self._servo_angles = {}

    def _receive_joint_cmd(self, data):
        joint_cmd = joint_cmd_pb2.JointCmd.FromString(data)
        index = self._servo_names.index(self.joint_cmd.name)
        if index < 0:
            return

        if (not joint_cmd.HasField('position') or
            not joint_cmd.position.HasField('target')):
            return

        self._servo_angles[index] = joint_cmd.position.target

    @asyncio.coroutine
    def set_pose(self, id_to_deg_map, pose_time=None):
        for ident, angle in id_to_deg_map.iteritems():
            self.joint_cmd.name = self._servo_names[ident]
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
def servo_controller(servo_type, **kwargs):
    if servo_type == 'herkulex':
        if 'model_name' in kwargs:
            kwargs.pop('model_name')
        result = HerkuleXController(**kwargs)
    elif servo_type == 'gazebo':
        if 'serial_port' in kwargs:
            kwargs.pop('serial_port')
        result = GazeboController(**kwargs)
    else:
        raise RuntimeError('unknown servo type: ' + servo_type)
    yield From(result.start())
    raise Return(result)
