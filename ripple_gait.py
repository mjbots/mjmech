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


class RippleConfig(object):
    def __init__(self):
        self.mechanical = MechanicalConfig()

        self.max_cycle_time_s = 4.0
        self.lift_height_mm = 80.0
        self.swing_percent = 100.0
        self.position_margin_percent = 80.0
        self.leg_order = []
        self.body_z_offset_mm = 0.0
        self.servo_speed_margin_percent = 70.0

    def copy(self):
        result = RippleConfig()
        result.mechanical = self.mechanical.copy()
        result.max_cycle_time_s = self.max_cycle_time_s
        result.lift_height_mm = self.lift_height_mm
        result.swing_percent = self.swing_percent
        result.position_margin_percent = self.position_margin_percent
        result.leg_order = self.leg_order[:]
        result.body_z_offset_mm = self.body_z_offset_mm
        result.servo_speed_margin_percent = self.servo_speed_margin_percent

        return result


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
        self.point = leg_ik.Point3D(0., 0., 0.)
        self.mode = STANCE
        self.frame = None
        self.shoulder_frame = None

    def __repr__(self):
        return '<LegState p=%r m=%d f=%s>' % (self.point, self.mode, self.frame)

class RippleState(object):
    def __init__(self):
        self.legs = {}
        self.phase = 0.
        self.action = 0

        # robot_frame coordinates describing the start and end
        # position of the current swing leg.
        self.swing_start_pos = leg_ik.Point3D()
        self.swing_end_pos = leg_ik.Point3D()

        self.world_frame = tf.Frame()
        self.robot_frame = tf.Frame(None, None, self.world_frame)
        self.body_frame = tf.Frame(None, None, self.robot_frame)

    def copy(self):
        result = RippleState()
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
            result.legs[key] = leg

        result.phase = self.phase
        result.action = self.action
        result.swing_start_pos = self.swing_start_pos.copy()
        result.swing_end_pos = self.swing_end_pos.copy()

        result.world_frame.transform = self.world_frame.transform.copy()
        result.robot_frame.transform = self.robot_frame.transform.copy()
        result.body_frame.transform = self.body_frame.transform.copy()

        return result

def _sign(val):
    return -1.0 if (val < 0.0) else 1.0

class NotSupported(RuntimeError):
    '''The given command is not realizable given the robot
    configuration.'''
    pass

class Options(object):
    cycle_time_s = 0.0
    servo_speed_dps = 0.0

class RippleGait(object):
    ACTION_START_SWING, ACTION_START_STANCE, ACTION_END = range(3)

    def __init__(self, config):
        self.config = config

        self.num_legs = len(config.mechanical.leg_config)

        self.cycle_time_s = None
        self.state = self.get_idle_state()
        self.idle_state = self.get_idle_state()
        self.next_command = None
        self.next_options = None

        self._create_actions()
        self._really_set_command(Command(), Options())

    def _create_actions(self):
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

        self.actions.append((1.0, -1, self.ACTION_END))



    def set_state(self, state, command):
        '''Force the current leg state to the given configuration.  If
        a phase is present, it must be consistent, i.e. it should have
        been read from this class along with the leg state.

        This may raise NotSupported, if the command and state are
        inconsistent with one another.  In this case, neither the
        state nor command are changed.
        '''

        old_state = self.state
        self.state = state.copy()

        # Make sure all the legs are in the correct frame.
        assert state.phase == 0.0

        for leg in self.state.legs.values():
            if leg.mode == STANCE:
                leg.point = self.state.world_frame.map_from_frame(
                    leg.frame, leg.point)
                leg.frame = self.state.world_frame
            elif leg.mode == SWING:
                leg.point = self.state.robot_frame.map_from_frame(
                    leg.frame, leg.point)
                leg.frame = self.state.robot_frame

        try:
            self.set_command(command)
        except:
            self.state = old_state
            raise

        return self.state

    def _select_command_options(self, command):
        if self.num_legs == 0:
            return Options()

        # First, iterate, solving IK for all legs in time until we
        # find the point at which the first leg is unsolvable.
        dt = 0.05

        time_s = 0.0

        my_state = self.idle_state.copy()
        self._apply_body_command(my_state, command)

        end_time_s = None
        min_observed_speed = None

        # Dictionary of (direction, leg_num) to old ik_result
        old_ik_result = {}

        fraction_in_stance = 1.0 - self._swing_phase_time()
        margin = 0.01 * self.config.position_margin_percent * fraction_in_stance

        while time_s < (0.5 * self.config.max_cycle_time_s / margin):
            if end_time_s is not None:
                break
            time_s += dt

            for direction in [-1, 1]:
                frame = self._get_update_frame(
                    direction * time_s, command=command)

                if end_time_s is not None:
                    break

                for leg_num, leg in my_state.legs.iteritems():
                    # TODO: Need to do this for the lifted leg as well.
                    leg_robot_frame_point = frame.map_to_parent(leg.point)
                    leg_shoulder_point = leg.shoulder_frame.map_from_frame(
                        my_state.robot_frame, leg_robot_frame_point)

                    leg_config = self.config.mechanical.leg_config[leg_num]
                    result = leg_config.leg_ik.do_ik(leg_shoulder_point)
                    if result is None:
                        # Break, so that we can take action knowing
                        # how far we can go.
                        end_time_s = time_s
                        break

                    if (direction, leg_num) in old_ik_result:
                        this_old_result = old_ik_result[(direction, leg_num)]

                        largest_change_deg = \
                            leg_config.leg_ik.largest_change_deg(
                            result, this_old_result)
                        this_speed = largest_change_deg / dt
                        if (min_observed_speed is None or
                            this_speed < min_observed_speed):
                            min_observed_speed = this_speed

                    old_ik_result[(direction, leg_num)] = result

        if min_observed_speed is None:
            raise NotSupported()

        result = Options()
        if end_time_s is None:
            # We can achieve this at the maximum time.
            result.cycle_time_s = self.config.max_cycle_time_s
        else:
            result.cycle_time_s = (2.0 * end_time_s * margin)

        # TODO jpieper: See if this cycle time is feasible.  We will
        # do this by checking to see if the swing leg has to move too
        # fast.
        min_swing_speed = (min_observed_speed *
                           (1.0 - self._swing_phase_time()) /
                           self._swing_phase_time())

        result.servo_speed_dps = min_swing_speed

        any_ik = self.config.mechanical.leg_config.values()[0].leg_ik
        servo_speed_dps = any_ik.servo_speed_dps()

        speed_margin = 0.01 * self.config.servo_speed_margin_percent
        if min_swing_speed > speed_margin * servo_speed_dps:
            # Slow the command down.
            slow_down_factor = (min_swing_speed /
                                (speed_margin * servo_speed_dps))
            command.translate_x_mm_s /= slow_down_factor
            command.translate_y_mm_s /= slow_down_factor
            command.rotate_deg_s /= slow_down_factor
            result.cycle_time_s *= slow_down_factor
            result.servo_speed_dps = speed_margin * servo_speed_dps

        return result

    def set_command(self, command):
        '''Set the current command.  This will raise a NotSupported
        exception if the platform cannot achieve the desired command,
        in this case, the desired command will not be changed.'''

        command = command.copy()

        # Determine if the command is valid or not, and select the
        # options necessary for it.
        #
        # NOTE: This may modify command.
        options = self._select_command_options(command)

        if self.state.phase != 0.0:
            # TODO jpieper: It would be nice to be able to update the
            # command anytime all the legs are in STANCE (or even
            # better, all the time), rather than just at phase 0.
            #
            # In either case, you have to be careful that the new
            # action list is consistent with the new command and phase
            # or do something more complicated like interpolate
            # between the two commands.
            self.next_command = command
            self.next_options = options
            return

        self._really_set_command(command, options)

    def _really_set_command(self, command, options):
        self.command = command
        self.options = options

        if self.num_legs == 0:
            return

        self.cycle_time_s = options.cycle_time_s
        self._apply_body_command(self.state, command)


    def _apply_body_command(self, state, command):
        state.body_frame.transform.translation.x = command.body_x_mm
        state.body_frame.transform.translation.y = command.body_y_mm
        state.body_frame.transform.translation.z = (
            command.body_z_mm + self.config.body_z_offset_mm)
        state.body_frame.transform.rotation = tf.Quaternion.from_euler(
            math.radians(command.body_roll_deg),
            math.radians(command.body_pitch_deg),
            math.radians(command.body_yaw_deg))


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
            leg_state.point = result.world_frame.map_from_frame(
                result.body_frame, point)
            leg_state.frame = result.world_frame
            result.legs[leg_number] = leg_state

            # For now, we are assuming that shoulders face away from
            # the y axis.
            rotation = (0.5 * math.pi
                        if leg_config.mount_x_mm > 0.0
                        else -0.5 * math.pi)
            leg_state.shoulder_frame = tf.Frame(
                tf.Point3D(leg_config.mount_x_mm,
                           leg_config.mount_y_mm,
                           leg_config.mount_z_mm),
                tf.Quaternion.from_euler(0., 0., rotation),
                result.body_frame)

        result.body_frame.transform.translation.z = self.config.body_z_offset_mm

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
        return self.advance_phase(delta_s / self._phase_time())

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

    def _get_update_frame(self, dt, command=None):
        if command is None:
            command = self.command

        update_frame = tf.Frame(parent=self.state.robot_frame)
        vx = command.translate_x_mm_s
        vy = command.translate_y_mm_s
        if command.rotate_deg_s == 0:
            dx = vx * dt
            dy = vy * dt
        else:
            vyaw = math.radians(command.rotate_deg_s)
            dx = (((math.cos(dt * vyaw) - 1) * vy +
                   math.sin(dt * vyaw) * vx) / vyaw)
            dy = (((math.cos(dt * vyaw) - 1) * vx +
                   math.sin(dt * vyaw) * vy) / vyaw)
            update_frame.transform.rotation = \
                tf.Quaternion.from_euler(0, 0, dt * vyaw)

        update_frame.transform.translation.x = dx
        update_frame.transform.translation.y = dy
        return update_frame


    def _noaction_advance_phase(self, delta_phase, final_phase):
        transform = self.state.robot_frame.transform
        dt = delta_phase * self._phase_time()

        update_frame = self._get_update_frame(dt)

        new_transform = update_frame.transform_to_frame(self.state.world_frame)
        self.state.robot_frame.transform = new_transform

        # Update the legs which are in swing.
        for leg in self.state.legs.values():
            if leg.mode == SWING:
                leg_phase = ((final_phase % (1.0 / self.num_legs)) /
                             self._swing_phase_time())
                # Don't allow the phase to wrap-around on the final update.
                if delta_phase > 0.0 and leg_phase == 0.0:
                    leg_phase = 1.0
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
            return self.state

        old_phase = self.state.phase
        next_phase = (self.state.phase + delta_phase)

        cur_phase = old_phase

        while True:
            # Are there any actions between old_phase and new_phase?
            if self.actions[self.state.action][0] > next_phase:
                break

            advance_phase = self.actions[self.state.action][0]
            if advance_phase != cur_phase:
                delta_phase = advance_phase - cur_phase
                self._noaction_advance_phase(delta_phase, advance_phase)

            cur_phase = advance_phase
            self._do_action(self.state.action)

            self.state.action += 1

            if self.state.action >= len(self.actions):
                self.state.action = 0
                next_phase -= 1.0
                cur_phase -= 1.0

                if self.next_command:
                    next_command = self.next_command
                    self.next_command = None
                    next_options = self.next_options
                    self.next_options = None
                    self._really_set_command(next_command, next_options)

        # Finally, advance the remainder of the phase and update the phase.
        self._noaction_advance_phase(next_phase - cur_phase, next_phase)
        self.state.phase = next_phase

        return self.state

    def _phase_time(self):
        return self.cycle_time_s

    def _swing_phase_time(self):
        return (1.0 / self.num_legs) * 0.01 * self.config.swing_percent

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

        end_frame = self._get_update_frame(dt)

        # TODO jpieper: This should map from whatever frame the idle
        # state leg was actually in.
        return end_frame.map_to_parent(self.idle_state.legs[leg_num].point)
