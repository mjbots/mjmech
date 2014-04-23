# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import bisect
import copy
import math

import leg_ik
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
        self.frame = None

    def __repr__(self):
        return '<LegState p=%r m=%d f=%s>' % (self.point, self.mode, self.frame)

class RippleState(object):
    def __init__(self):
        self.legs = {}
        self.phase = 0.

        # robot_frame coordinates describing the start and end
        # position of the current swing leg.
        self.swing_start_pos = leg_ik.Point3D()
        self.swing_end_pos = leg_ik.Point3D()

        self.world_frame = tf.Frame()
        self.robot_frame = tf.Frame(None, None, self.world_frame)
        self.body_frame = tf.Frame(None, None, self.robot_frame)

def _sign(val):
    return -1.0 if (val < 0.0) else 1.0

class NotSupported(RuntimeError):
    '''The given command is not realizable given the robot
    configuration.'''
    pass

class RippleGait(object):
    ACTION_START_SWING, ACTION_START_STANCE, ACTION_END = range(3)

    def __init__(self, config):
        self.config = config

        self.num_legs = len(config.mechanical.leg_config)

        self.state = self.get_idle_state()
        self.idle_state = self.get_idle_state()
        self.set_command(Command())

    def set_state(self, state, command):
        '''Force the current leg state to the given configuration.  If
        a phase is present, it must be consistent, i.e. it should have
        been read from this class along with the leg state.

        This may raise NotSupported, if the command and state are
        inconsistent with one another.  In this case, neither the
        state nor command are changed.
        '''

        old_state = self.state
        self.state = copy.deepcopy(state)
        assert state.phase == 0.0
        for leg in self.state.legs.values():
            assert leg.mode == STANCE # Because we are at phase 0.0

            # Ensure that all legs, when in STANCE mode, are in the
            # world frame.
            leg.point = self.state.world_frame.map_from_frame(
                leg.frame, leg.point)
            leg.frame = self.state.world_frame

        try:
            self.set_command(command)
        except:
            self.state = old_state
            raise

    def set_command(self, command):
        '''Set the current command.  This will raise a NotSupported
        exception if the platform cannot achieve the desired command,
        in this case, the desired command will not be changed.'''

        self.command = command
        self.actions = []

        if self.num_legs == 0:
            return

        # Create the action list.
        swing_time = self._swing_phase_time()

        for i in range(self.num_legs):
            fraction = float(i) / self.num_legs
            leg_num = self.config.leg_order[i]
            self.actions.append(
                (fraction, leg_num, self.ACTION_START_SWING))
            self.actions.append(
                (fraction + swing_time, leg_num, self.ACTION_START_STANCE))

        self.actions.sort()
        self.actions.append((1.0, -1, self.ACTION_END))

        # TODO jpieper: Implement


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
            leg_state.frame = result.robot_frame
            result.legs[leg_number] = leg_state

        result.body_frame.transform.translation.z = self.config.body_z_offset

        return result

    def get_gait_graph(self):
        '''Return a GaitGraph instance for the current configuration
        and command.  This is independent of the current state, but is
        dependent upon the configuration and command.'''

        if self.num_legs == 0:
            return GaitGraph()

        # TODO jpieper: This could be generated from the actions list.

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

    def _do_action(self, action_index):
        phase, leg_num, action = self.actions[action_index]

        if action == self.ACTION_START_STANCE:
            leg = self.state.legs[leg_num]

            leg.mode = STANCE
            leg.point = self.state.world_frame.map_from_frame(
                leg.frame, leg.point)
            leg.point.z = 0.0
            leg.frame = self.state.world_frame
        elif action == self.ACTION_START_SWING:
            leg = self.state.legs[leg_num]

            leg.mode = SWING
            leg.point = self.state.robot_frame.map_from_frame(
                leg.frame, leg.point)
            leg.frame = self.state.robot_frame

            self.state.swing_start_pos = leg.point.copy()
            self.state.swing_end_pos = self._get_swing_end_pos(leg_num)

    def _noaction_advance_phase(self, delta_phase, final_phase):
        transform = self.state.robot_frame.transform
        dt = delta_phase * self._phase_time()
        transform.translation.x += self.command.translate_x_mm_s * dt
        transform.translation.y += self.command.translate_y_mm_s * dt
        transform.rotation = (
            tf.Quaternion.from_euler(
                0, 0, math.radians(self.command.rotate_deg_s * dt)) *
            transform.rotation)

        # Update the legs which are in swing.
        for leg in self.state.legs.values():
            if leg.mode == SWING:
                leg_phase = ((final_phase % (1.0 / self.num_legs)) /
                             self._swing_phase_time())
                assert leg.frame is self.state.robot_frame
                delta = self.state.swing_end_pos - self.state.swing_start_pos
                current = self.state.swing_start_pos + delta.scaled(leg_phase)
                leg.point = current

                if leg_phase < 0.1:
                    leg.point.z = (leg_phase / 0.1) * self.config.lift_height_mm
                elif leg_phase < 0.9:
                    leg.point.z = self.config.lift_height_mm
                else:
                    leg.point.z = (((1.0 - leg_phase) / 0.1) *
                                   self.config.lift_height_mm)


    def advance_phase(self, delta_phase):
        '''Advance the phase and leg state by the given amount of
        phase.  Being independent of time, this is only really useful
        for visualization or debugging.'''
        if self.num_legs == 0:
            self.state.phase = (self.state.phase + delta_phase) % 1.0
            return

        old_phase = self.state.phase
        next_phase = (self.state.phase + delta_phase)

        cur_phase = old_phase
        action_index = bisect.bisect_left(self.actions, (cur_phase, 0, 0))
        if self.actions[action_index][0] < cur_phase:
            action_index += 1

        while True:
            # Are there any actions between old_phase and new_phase?
            if self.actions[action_index][0] < next_phase:
                self._do_action(action_index)
                delta_phase = self.actions[action_index][0] - cur_phase
                advance_phase = self.actions[action_index][0]
                self._noaction_advance_phase(delta_phase, advance_phase)
                cur_phase = advance_phase
                action_index += 1

                if action_index >= len(self.actions):
                    action_index = 0
                    next_phase -= 1.0
                    cur_phase = 0.0
            else:
                break

        # Finally, advance the remainder of the phase and update the phase.
        next_phase = next_phase % 1.0
        self._noaction_advance_phase(next_phase - cur_phase, next_phase)
        self.state.phase = next_phase

        return self.state

    def _phase_time(self):
        # TODO jpieper: Implement varying cycle times.
        return self.config.max_cycle_time_s

    def _swing_phase_time(self):
        # TODO jpieper: Use the configuration to select this.
        return 1.0 / self.num_legs

    def _guess_phase(self):
        '''Attempt to fill in the phase member, and any UNKNOWN
        LegState fields.'''
        raise NotImplementedError()

    def _get_swing_end_pos(self, leg_num):
        # Target swing end positions such that during stance, the leg
        # will spend half its travel time reaching the idle position,
        # and half its travel time going beyond the idle position.

        stance_phase_time = 1.0 - self._swing_phase_time()
        dt = 0.5 * stance_phase_time * self._phase_time()

        translation = tf.Point3D(self.command.translate_x_mm_s * dt,
                                 self.command.translate_y_mm_s * dt,
                                 0.0)
        rotation = tf.Quaternion.from_euler(
            0, 0, math.radians(self.command.rotate_deg_s * dt))
        end_frame = tf.Frame(translation, rotation)

        return end_frame.map_to_parent(self.idle_state.legs[leg_num].point)
