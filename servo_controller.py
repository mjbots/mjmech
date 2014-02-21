#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import pygazebo

import herkulex

# Constants used in the 'enable_power' method.
(# No power is applied, servos can be manipulated
 POWER_FREE,
 # Resistance is applied, but not actively controlled
 POWER_BRAKE,
 # Power is used to maintain position
 POWER_ENABLE) = range(3)

class HerculeXController(object):

    def __init__(self, serial_port=None):
        if serial_port is None:
            raise RuntimeError('serial port required')

        self.port = herkulex.HerkuleX(serial_port=serial_port)
        self.default_pose_time = 0.03

    def _angle_to_counts(self, angle_deg):
        return min(1023, max(0, int(512 + angle_deg / 0.325)))

    def _counts_to_angle(self, counts):
        return (counts - 512) * 0.325

    def set_pose(self, id_to_deg_map, pose_time=None):
        if pose_time is None:
            pose_time = self.default_pose_time

        self.port.s_jog(
            time_ms=pose_time * 1000,
            targets=[(ident, self._angle_to_counts(angle), 0)
                     for ident, angle in id_to_deg_map.iteritems()])

    def set_single_pose(self, ident, angle_deg, pose_time=None):
        self.set_pose({ident: angle_deg}, pose_time)

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
            self.port.ram_write(
                ident, self.port.REG_TORQUE_CONTROL, value)

    def get_pose(self, idents=[]):
        """Determine the current pose of the requested servos.

        :returns: a dictionary mapping identifier to angle in degrees
        """
        result = {}

        for ident in idents:
            counts = self.port.position(ident)
            result[ident] = self._counts_to_angle(counts)

        return result

    def get_torque(self, idents=[]):
        """Determine the current torque applied by each servo.

        :returns: a dictionary mapping identifier to torque in N*m
        """
        result = {}

        for ident in idents:
            pwm = self.port.pwm(ident)
            result[ident] = self._pwm_to_torque(pwm)

        return result

class GazeboController(object):
    def __init__(self, model_name):
        self.manager = pygazebo.Manager()

        self.publisher = self.manager.advertise(
            '/gazebo/default/%s/joint_cmd', 'gazebo.msgs.JointCmd')

        self.subscriber = self.manager.subscribe(
            '/gazebo/default/%s/joint_cmd', 'gazebo.msgs.JointCmd',
            self._receive_joint_cmd)

        self._servo_names = reduce(
            lambda a, b: a + b,
            [ [ '%s::coxa_hinge%d' % (model_name, x),
                '%s::femur_hinge%d' % (model_name, x),
                '%s::tibia_hinge%d' % (model_name, x) ]
              for x in range(4) ])

        self.joint_cmd = joint_cmd_pb2.JointCmd()
        self.joint_cmd.axis = 0
        self.joint_cmd.position.target = 0
        self.joint_cmd.position.p_gain = 140.0
        self.joint_cmd.position.i_gain = 1.0
        self.joint_cmd.position.d_gain = 2.0

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

    def set_pose(self, id_to_deg_map):
        for ident, angle in id_to_deg_map.iteritems():
            self.joint_cmd.name = self.servo_names[ident]
            self.joint_cmd.position.target = angle
            self.publisher.publish(self.joint_cmd)
            self._servo_angles[ident] = angle

    def get_pose(self, idents=[]):
        """Determine the current pose of the requested servos.

        :returns: a dictionary mapping identifier to angle in degrees
        """
        result = {}

        for ident in idents:
            result[ident] = self._servo_angles.get(ident)

        return result

    def get_torque(self, idents=[]):
        """Determine the current torque applied by each servo.

        :returns: a dictionary mapping identifier to torque in N*m
        """
        result = {}

        for ident in idents:
            result[ident] = None

        return result

    def set_single_pose(self, ident, angle_deg, pose_time=None):
        self.set_pose({ident: angle_deg}, pose_time)

    def enable_power(self, state, idents=None):
        """Enable the power state of one or more servos.

        :param value: one of POWER_FREE, POWER_BRAKE, or POWER_ENABLE
        :param idents: optional list of identifiers to configure, if
            None, all servos are configured
        """
        pass

def servo_controller(servo_type, **kwargs):
    if servo_type == 'herkulex':
        return HerculeXController(**kwargs)
    elif servo_type == 'gazebo':
        return GazeboController(**kwargs)
    else:
        raise RuntimeError('unknown servo type: ' + servo_type)
