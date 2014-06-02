# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import copy

import tf

# These are possible modes for each of the legs.

# In this phase, the leg is intended to be fixed relative to the
# ground.
STANCE = 0

# In this phase, the leg is not intended to be in contact with the
# ground.
SWING = 1

# This mode is used when the current mode is not known.
UNKNOWN = 2


class LegConfig(object):
    # The mounting positions are relative to the geometric center of
    # the body.
    mount_x_mm = 0
    mount_y_mm = 0
    mount_z_mm = 0

    # The idle positions are measured relative to the ground under
    # each mounting location, with the unique property that positive x
    # is defined always as away from the midline and y is defined as
    # away from the centerline.  (This means that it is valid for all
    # leg's idle states to be the same).
    idle_x_mm = 100
    idle_y_mm = 0
    idle_z_mm = 0

    leg_ik = None

    def copy(self):
        result = LegConfig()
        result.mount_x_mm = self.mount_x_mm
        result.mount_y_mm = self.mount_y_mm
        result.mount_z_mm = self.mount_z_mm

        result.idle_x_mm = self.idle_x_mm
        result.idle_y_mm = self.idle_y_mm
        result.idle_z_mm = self.idle_z_mm

        result.leg_ik = self.leg_ik
        return result

class MechanicalConfig(object):
    def __init__(self):
        # A map from leg number to LegConfig instances.
        self.leg_config = {}

        # The center of gravity relative to the geometric center.
        self.body_cog_x_mm = None
        self.body_cog_y_mm = None
        self.body_cog_z_mm = None

    def copy(self):
        result = MechanicalConfig()
        for leg_num, leg in self.leg_config.iteritems():
            result.leg_config[leg_num] = leg.copy()
        result.body_cog_x_mm = self.body_cog_x_mm
        result.body_cog_y_mm = self.body_cog_y_mm
        result.body_cog_z_mm = self.body_cog_z_mm

        return result

class LegResult(object):
    '''This gives the relative position of the leg to the shoulder
    joint where it mounts onto the body.'''
    def __init__(self):
        self.point = tf.Point3D(0., 0., 0.)
        self.mode = STANCE

class Command(object):
    translate_x_mm_s = 0.0
    translate_y_mm_s = 0.0
    rotate_deg_s = 0.0
    body_x_mm = 0.0
    body_y_mm = 0.0
    body_z_mm = 0.0
    body_pitch_deg = 0.0
    body_roll_deg = 0.0
    body_yaw_deg = 0.0

    def copy(self):
        return copy.copy(self)


class GaitGraphLeg(object):
    sequence = None
    '''A list of tuples (phase, mode)
        phase - the gait phase where this mode starts
        mode - a leg mode describing what the leg starts doing
               at this time
    '''

class GaitGraph(object):
    def __init__(self):
        # This is a mapping of leg numbers to GaitGraphLeg instances.
        self.leg = {}

class LegState(object):
    '''This keeps track of the robot position coordinates of each
    leg.'''
    def __init__(self):
        self.point = tf.Point3D(0., 0., 0.)
        self.mode = STANCE
        self.frame = None
        self.shoulder_frame = None
        self.leg_ik = None

    def __repr__(self):
        return '<LegState p=%r m=%d f=%s>' % (self.point, self.mode, self.frame)

class NotSupported(RuntimeError):
    '''The given command is not realizable given the robot
    configuration.'''
    pass

class CommonState(object):
    def __init__(self):
        self.legs = {}

        self.world_frame = tf.Frame()
        self.robot_frame = tf.Frame(None, None, self.world_frame)
        self.body_frame = tf.Frame(None, None, self.robot_frame)
        self.cog_frame = tf.Frame(None, None, self.body_frame)

    def copy_into(self, result):
        for key, value in self.legs.iteritems():
            leg = LegState()
            leg.point = value.point.copy()
            leg.mode = value.mode
            if value.frame is self.world_frame:
                leg.frame = result.world_frame
            elif value.frame is self.robot_frame:
                leg.frame = result.robot_frame
            elif value.frame is self.body_frame:
                leg.frame = result.body_frame
            leg.shoulder_frame = tf.Frame(
                value.shoulder_frame.transform.translation.copy(),
                value.shoulder_frame.transform.rotation.copy(),
                result.body_frame)
            leg.leg_ik = value.leg_ik
            result.legs[key] = leg

        result.world_frame.transform = self.world_frame.transform.copy()
        result.robot_frame.transform = self.robot_frame.transform.copy()
        result.body_frame.transform = self.body_frame.transform.copy()
        result.cog_frame.transform = self.cog_frame.transform.copy()

    def command_dict(self):
        result = {}
        for leg in self.legs.itervalues():
            shoulder_point = leg.shoulder_frame.map_from_frame(
                leg.frame, leg.point)
            ik_result = leg.leg_ik.do_ik(shoulder_point)
            if ik_result is None:
                raise NotSupported()
            result.update(ik_result.command_dict())
        return result
