# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import leg_ik

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
    mount_x_mm = None
    mount_y_mm = None
    mount_z_mm = None

    # The idle positions are measured relative to the ground under
    # each mounting location, with the unique property that positive x
    # is defined always as away from the midline and y is defined as
    # away from the centerline.  (This means that it is valid for all
    # leg's idle states to be the same).
    idle_x_mm = None
    idle_y_mm = None
    idle_z_mm = None

    leg_ik = None

class MechanicalConfig(object):
    def __init__(self):
        # A map from leg number to LegConfig instances.
        self.leg_config = {}

        # The center of gravity relative to the geometric center.
        self.body_cog_x_mm = None
        self.body_cog_y_mm = None
        self.body_cog_z_mm = None

class RippleConfig(object):
    def __init__(self):
        self.mechanical = MechanicalConfig()

        self.max_cycle_time_s = None
        self.lift_height_mm = None
        self.min_swing_percent = None
        self.max_swing_percent = None
        self.leg_order = None
        self.body_z_offset = None


class LegResult(object):
    '''This gives the relative position of the leg to the shoulder
    joint where it mounts onto the body.'''
    def __init__(self):
        self.point = leg_ik.Point3D(0., 0., 0.)
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
        self.point = leg_ik.Point3D(0., 0., 0.)
        self.mode = STANCE

class RippleState(object):
    def __init__(self):
        self.legs = {}
        self.robot_position_mm = leg_ik.Point3D(0., 0., 0.)
        self.robot_heading_rad = 0.
        self.phase = 0.

def _sign(val):
    return -1.0 if (val < 0.0) else 1.0

class NotSupported(RuntimeError):
    '''The given command is not realizable given the robot
    configuration.'''
    pass

class RippleGait(object):
    def __init__(self, config):
        self.config = config

        self.num_legs = len(config.mechanical.leg_config)

        self.command = Command()

        self.state = self.get_idle_state()


    def set_state(self, state, command):
        '''Force the current leg state to the given configuration.  If
        a phase is present, it must be consistent, i.e. it should have
        been read from this class along with the leg state.

        This may raise NotSupported, if the command and state are
        inconsistent with one another.  In this case, neither the
        state nor command are changed.
        '''

        old_state = self.state
        self.state = state

        try:
            self.set_command(command)
        except:
            self.state = old_state
            raise

    def set_command(self, command):
        '''Set the current command.  This will raise a NotSupported
        exception if the platform cannot achieve the desired command,
        in this case, the desired command will not be changed.'''
        raise NotImplementedError()

    def get_idle_state(self):
        '''Return a state usable for the idle stance, regardless of
        the current state of the gait generator.'''
        result = RippleState()

        for leg_data in self.config.mechanical.leg_config.iteritems():
            leg_number, leg_config = leg_data

            point = leg_ik.Point3D(0., 0., 0.)

            x_sign = _sign(leg_config.mount_x_mm)
            point.x = leg_config.mount_x_mm + leg_config.idle_x_mm * x_sign

            y_sign = _sign(leg_config.mount_y_mm)
            point.y = leg_config.mount_y_mm + leg_config.idle_y_mm * y_sign

            point.z = leg_config.idle_z_mm

            leg_state = LegState()
            leg_state.point = point
            result.legs[leg_number] = leg_state

        return result

    def get_gait_graph(self):
        '''Return a GaitGraph instance for the current configuration
        and command.  This is independent of the current state, but is
        dependent upon the configuration and command.'''

        start_phase = 0.0
        leg_cycle_time = 1.0 / self.num_legs

        result = GaitGraph()
        for leg_number in self.config.leg_order:
            graph_leg = GaitGraphLeg()

            graph_leg.sequence = [
                (start_phase, SWING),
                (start_phase + self._swing_phase_time(), STANCE)]
            if start_phase != 0:
                graph_leg.sequence = [(0.0, STANCE)] + graph_leg.sequence

            result.leg[leg_number] = graph_leg

            start_phase += leg_cycle_time

        return result

    def advance_time(self, delta_s):
        '''Advance the phase and leg state by the given amount of
        time.'''
        raise NotImplementedError()

    def advance_phase(self, delta_phase):
        '''Advance the phase and leg state by the given amount of
        phase.  Being independent of time, this is only really useful
        for visualization or debugging.'''
        raise NotImplementedError()

    def _swing_phase_time(self):
        # TODO jpieper: Use the configuration to select this.
        return 1.0 / self.num_legs

    def _guess_phase(self):
        '''Attempt to fill in the phase member, and any UNKNOWN
        LegState fields.'''
        raise NotImplementedError()
