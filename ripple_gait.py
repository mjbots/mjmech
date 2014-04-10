# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import leg_ik

class LegConfig(object):
    number = None

    # The mounting positions are relative to the geometric center of
    # the body.
    mount_x_mm = None
    mount_y_mm = None
    mount_z_mm = None

    # The idle positions are measured relative to the mounting
    # location, with the unique property that positive x is defined
    # always as away from the midline and y is defined as away from
    # the centerline.  (This means that it is valid for all leg's idle
    # states to be the same).
    idle_x_mm = None
    idle_y_mm = None
    idle_z_mm = None

    leg_ik = None

class MechanicalConfig(object):
    def __init__(self):
        # A list of LegConfig instances.
        self.leg_config = []

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


class LegState(object):
    STANCE, SWING, UNKNOWN = range(3)

    def __init__(self):
        self.point = leg_ik.Point3D()
        self.mode = self.STANCE

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
        mode - a LegState.mode describing what the leg starts doing at this time
    '''

class GaitGraph(object):
    def __init__(self):
        # This is a mapping of leg numbers to GaitGraphLeg instances.
        self.leg = {}

class RippleGait(object):
    def __init__(self, config):
        self.config = config

        self.num_legs = len(config.mechanical.leg_config)

        self.command = Command()

        # This is a mapping from LegConfig.number to LegState
        # instance.

        # TODO jpieper: This should be initialized to the idle stance
        # based on the configuration.
        self.leg_state = {}
        self.phase = 0.0


    def set_leg_state(self, leg_state, phase=None):
        '''Force the current leg state to the given configuration.  If
        a phase is present, it must be consistent, i.e. it should have
        been read from this class along with the leg state.'''

        self.leg_state = leg_state
        self.phase = phase

        self._guess_phase_and_mode()

    def set_command(self, command):
        '''Set the current command.  This will raise a NotSupported
        exception if the platform cannot achieve the desired
        command.'''
        raise NotImplementedError()

    def get_idle_state(self):
        '''Return the idle leg state, regardless of what the current
        state of the gait generator is.'''
        raise NotImplementedError()

    def get_gait_graph(self):
        '''Return a GaitGraph instance for the current configuration
        and command.'''
        raise NotImplementedError()

    def advance_time(self, delta_s):
        '''Advance the phase and leg state by the given amount of
        time.'''
        raise NotImplementedError()

    def advance_phase(self, delta_phase):
        '''Advance the phase and leg state by the given amount of
        phase.  Being independent of time, this is only really useful
        for visualization or debugging.'''
        raise NotImplementedError()

    def _guess_phase(self):
        '''Attempt to fill in the phase member, and any UNKNOWN
        LegState fields.'''
        raise NotImplementedError()
