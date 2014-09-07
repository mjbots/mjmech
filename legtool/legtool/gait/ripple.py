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

'''An implementation of a simple open loop ripple gait, with optional
grouping of legs.'''

import bisect
import math

from .common import (STANCE, SWING, UNKNOWN)
from .common import (LegConfig, MechanicalConfig)
from .common import (LegResult, Command, GaitGraphLeg, GaitGraph, LegState)
from .common import (NotSupported, CommonState)
from ..tf import geometry
from ..tf import tf

class RippleConfig(object):
    def __init__(self):
        self.mechanical = MechanicalConfig()

        self.max_cycle_time_s = 4.0
        self.lift_height_mm = 80.0
        self.lift_percent = 25.0
        self.swing_percent = 80.0
        self.position_margin_percent = 80.0
        self.leg_order = []
        self.body_z_offset_mm = 0.0
        self.servo_speed_margin_percent = 70.0
        self.statically_stable = False
        self.static_center_factor = 3.0
        self.static_stable_factor = 10.0
        self.static_margin_mm = 20.0

    def copy(self):
        result = RippleConfig()
        result.mechanical = self.mechanical.copy()
        result.max_cycle_time_s = self.max_cycle_time_s
        result.lift_height_mm = self.lift_height_mm
        result.lift_percent = self.lift_percent
        result.swing_percent = self.swing_percent
        result.position_margin_percent = self.position_margin_percent
        result.leg_order = self.leg_order[:]
        result.body_z_offset_mm = self.body_z_offset_mm
        result.servo_speed_margin_percent = self.servo_speed_margin_percent
        result.statically_stable = self.statically_stable
        result.static_center_factor = self.static_center_factor
        result.static_stable_factor = self.static_stable_factor
        result.static_margin_mm = self.static_margin_mm

        return result

    @staticmethod
    def parse_leg_order(data):
        '''A leg ordering is a comma separated list of leg numbers, or
        of leg groups, where a leg group is a parenthesis grouped list
        of leg numbers.

        Return the programmatic representation of that ordering when
        given a string version.  On malformed input, make all attempts
        to return something, even if only a subset of the input.
        '''
        result = []
        if data == '':
            return result

        in_tuple = False
        current_tuple = ()
        current_item = ''
        for x in data:
            if x == '(':
                if in_tuple:
                    return result
                in_tuple = True

            if x >= '0' and x <= '9':
                current_item += x
            else:
                if len(current_item):
                    value = int(current_item)
                    current_item = ''
                    if in_tuple:
                        current_tuple += (value,)
                    else:
                        result.append(value)
                if x == ')':
                    if not in_tuple:
                        return result
                    if len(current_tuple) == 1:
                        result.append(current_tuple[0])
                    elif len(current_tuple) > 1:
                        result.append(current_tuple)
                    current_tuple = ()
                    in_tuple = False

        if len(current_item):
            result.append(int(current_item))

        return result

    @staticmethod
    def str_leg_order(data):
        '''Given a leg ordering, return the canonical string
        representation.'''
        assert isinstance(data, list)
        return str(data)[1:-1].replace(' ', '')

    _FLOAT_ATTRIBUTES = [
        'max_cycle_time_s',
        'lift_height_mm',
        'lift_percent',
        'swing_percent',
        'position_margin_percent',
        'body_z_offset_mm',
        'servo_speed_margin_percent',
        'static_center_factor',
        'static_stable_factor',
        'static_margin_mm',
        ]

    @staticmethod
    def read_settings(config, group_name, leg_ik_map):
        '''Populate a RippleConfig instance from the given
        ConfigParser instance and group name.

        :param config: Configuration to read
        :param group_name: String containing the appropriate group
        :param leg_ik_map: Mapping from leg number to IK instance'''
        result = RippleConfig()

        result.mechanical = MechanicalConfig.read_settings(
            config, group_name + '.legs', leg_ik_map)

        for x in RippleConfig._FLOAT_ATTRIBUTES:
            if config.has_option(group_name, x):
                setattr(result, x, config.getfloat(group_name, x))

        if config.has_option(group_name, 'statically_stable'):
            result.statically_stable = config.getboolean(
                group_name, 'statically_stable')

        if config.has_option(group_name, 'leg_order'):
            result.leg_order = RippleConfig.parse_leg_order(
                config.get(group_name, 'leg_order'))

        return result

    def write_settings(self, config, group_name):
        '''Store this RippleConfig instance into the given
        ConfigParser instance at the given group name.'''

        config.add_section(group_name)
        self.mechanical.write_settings(config, group_name + '.legs')

        for x in self._FLOAT_ATTRIBUTES:
            config.set(group_name, x, getattr(self, x))

        config.set(group_name, 'statically_stable', self.statically_stable)
        config.set(group_name, 'leg_order', self.str_leg_order(self.leg_order))

class RippleState(CommonState):
    def __init__(self):
        self.legs = {}
        self.phase = 0.
        self.action = 0

        # robot_frame coordinates describing the start and end
        # position of the current swing leg(s).
        self.swing_start_pos = {}
        self.swing_end_pos = {}

        self.world_frame = tf.Frame()
        self.robot_frame = tf.Frame(None, None, self.world_frame)
        self.body_frame = tf.Frame(None, None, self.robot_frame)
        self.cog_frame = tf.Frame(None, None, self.body_frame)

    def copy(self):
        result = RippleState()
        super(RippleState, self).copy_into(result)

        result.phase = self.phase
        result.action = self.action
        result.swing_start_pos = dict(
            [(key, value.copy()) for key, value in
             self.swing_start_pos.iteritems()])
        result.swing_end_pos = dict(
            [(key, value.copy()) for key, value in
             self.swing_end_pos.iteritems()])

        return result

def _sign(val):
    return -1.0 if (val < 0.0) else 1.0

class Options(object):
    cycle_time_s = 0.0
    servo_speed_dps = 0.0

def _iterate_legs(leg_group):
    """Given a leg group (either a scalar leg number, or a tuple of
    legs), iterate over all of them."""
    if isinstance(leg_group, int):
        yield leg_group
    else:
        for x in leg_group:
            yield x

class RippleGait(object):
    ACTION_START_SWING, ACTION_START_STANCE, ACTION_END = range(3)

    def __init__(self, config):
        assert config is not None
        self.config = config

        self.num_legs = len(config.leg_order)

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
            leg_group = self.config.leg_order[i]
            self.actions.append(
                (fraction, leg_group, self.ACTION_START_SWING))
            self.actions.append(
                (fraction + swing_time, leg_group, self.ACTION_START_STANCE))

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

    def _do_commands_differ_body_only(self, command1, command2):
        return (command1.translate_x_mm_s == command2.translate_x_mm_s and
                command1.translate_y_mm_s == command2.translate_y_mm_s and
                command1.rotate_deg_s == command2.rotate_deg_s)

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

        self._really_set_command(command, options)

    def is_command_pending(self):
        return self.next_command is not None

    def _really_set_command(self, command, options):
        self.command = command
        self.options = options

        if self.num_legs == 0:
            return

        self.cycle_time_s = options.cycle_time_s
        self._apply_body_command(self.state, command)


    def _apply_body_command(self, state, command):
        if not self.config.statically_stable:
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

            point = tf.Point3D(0., 0., 0.)

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
            leg_state.leg_ik = leg_config.leg_ik

        result.body_frame.transform.translation.z = self.config.body_z_offset_mm
        result.cog_frame.transform.translation.x = \
            self.config.mechanical.body_cog_x_mm
        result.cog_frame.transform.translation.y = \
            self.config.mechanical.body_cog_y_mm
        result.cog_frame.transform.translation.z = \
            self.config.mechanical.body_cog_z_mm

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
        for leg_group in self.config.leg_order:
            graph_leg = GaitGraphLeg()

            graph_leg.sequence = [
                (start_phase, SWING),
                (start_phase + self._swing_phase_time(), STANCE)]
            if start_phase != 0:
                graph_leg.sequence = [(0.0, STANCE)] + graph_leg.sequence

            for leg_number in _iterate_legs(leg_group):
                result.leg[leg_number] = graph_leg

            start_phase += leg_cycle_time

        return result

    def advance_time(self, delta_s):
        '''Advance the phase and leg state by the given amount of
        time.'''
        return self.advance_phase(delta_s / self._phase_time())

    def _do_action(self, action_index):
        phase, leg_group, action = self.actions[action_index]

        if action == self.ACTION_START_STANCE:
            for leg_num in _iterate_legs(leg_group):
                leg = self.state.legs[leg_num]

                leg.mode = STANCE
                leg.point = self.state.world_frame.map_from_frame(
                    leg.frame, leg.point)
                leg.point.z = 0.0
                leg.frame = self.state.world_frame
        elif action == self.ACTION_START_SWING:
            for leg_num in _iterate_legs(leg_group):
                leg = self.state.legs[leg_num]

                leg.mode = SWING
                leg.point = self.state.robot_frame.map_from_frame(
                    leg.frame, leg.point)
                leg.frame = self.state.robot_frame

                self.state.swing_start_pos[leg_num] = leg.point.copy()
                self.state.swing_end_pos[leg_num] = (
                    self._get_swing_end_pos(leg_num))

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
        for leg_num, leg in self.state.legs.iteritems():
            if leg.mode == SWING:
                leg_phase = ((final_phase % (1.0 / self.num_legs)) /
                             self._swing_phase_time())
                # Don't allow the phase to wrap-around on the final update.
                if delta_phase > 0.0 and leg_phase == 0.0:
                    leg_phase = 1.0
                assert leg.frame is self.state.robot_frame
                delta = (self.state.swing_end_pos[leg_num] -
                         self.state.swing_start_pos[leg_num])
                current = (self.state.swing_start_pos[leg_num] +
                           delta.scaled(leg_phase))
                leg.point = current

                lift_fraction = 0.01 * self.config.lift_percent
                if leg_phase < lift_fraction:
                    leg.point.z = ((leg_phase / lift_fraction) *
                                   self.config.lift_height_mm)
                elif leg_phase < (1.0 - lift_fraction):
                    leg.point.z = self.config.lift_height_mm
                else:
                    leg.point.z = (((1.0 - leg_phase) / lift_fraction) *
                                   self.config.lift_height_mm)

        # If we are running with a statically stable gait, update the
        # body position dynamically.
        if self.config.statically_stable:
            desired_cog_vel = tf.Point3D()

            cog_pos = self.state.cog_frame.map_to_frame(
                self.state.robot_frame, tf.Point3D())
            cog_pos.z = 0

            # Do the part which pulls the COG back towards the center
            # of the robot frame.
            cog_dist = cog_pos.length()
            scale = 1.0
            if cog_dist < self.config.static_margin_mm:
                scale = cog_dist / self.config.static_margin_mm
            desired_cog_vel = tf.Point3D()

            if cog_dist != 0.0:
                static_center_velocity_mm_s = (
                    self.config.static_center_factor *
                    self.config.static_margin_mm /
                    self.options.cycle_time_s)
                desired_cog_vel = desired_cog_vel + cog_pos.scaled(
                    -scale * static_center_velocity_mm_s /
                     cog_dist)

            if (dt * desired_cog_vel.length()) > cog_dist:
                desired_cog_vel = desired_cog_vel.scaled(
                    cog_dist / (dt * desired_cog_vel.length()))

            # Now do the part which pulls the COG towards the support
            # polygon.

            # First, determine what support polygon we should be
            # aiming for.
            support_poly = self._get_current_support_poly_robot_frame()
            support_centroid = geometry.poly_centroid(support_poly)
            delta = cog_pos - support_centroid

            if not geometry.point_in_poly(cog_pos, support_poly):
                # We are not in the support polygon, thus move at the
                # maximal velocity towards the centroid
                scale = 1.0
            else:
                # We are in the support polygon.  See by how much to
                # determine what our scale should be.
                dist = geometry.distance_to_poly(cog_pos, support_poly)
                scale = max(0.0,
                            ((self.config.static_margin_mm - dist) /
                             self.config.static_margin_mm))

            static_stable_velocity_mm_s = (
                self.config.static_stable_factor *
                self.config.static_margin_mm /
                self.options.cycle_time_s)
            if delta.length() > 0.0:
                poly_vel = delta.scaled(
                    -scale * static_stable_velocity_mm_s / delta.length())
            else:
                poly_vel = tf.Point3D()
            if (dt * poly_vel.length()) > delta.length():
                poly_vel = poly_vel.scaled(
                    delta.length() / (dt * poly_vel.length()))

            cog_move = (desired_cog_vel + poly_vel).scaled(dt)
            self.state.body_frame.transform.translation.x += cog_move.x
            self.state.body_frame.transform.translation.y += cog_move.y


    def _get_current_support_poly_robot_frame(self):
        next_action = self.actions[self.state.action]

        _, action_leg_group, action = next_action

        if action == self.ACTION_END:
            _, action_leg_group, action = self.actions[0]

        legs = []
        if action == self.ACTION_START_SWING:
            # The next action is a lift, which means we should use all
            # legs but this one.
            legs = [x for x in self.state.legs.keys()
                    if x not in list(_iterate_legs(action_leg_group))]
        elif action == self.ACTION_START_STANCE:
            # We must have a leg in the air now, so use the current
            # set of stance legs.
            legs = [x for x in self.state.legs.keys()
                    if self.state.legs[x].mode == STANCE ]

        poly = [self.state.robot_frame.map_from_frame(
                self.state.legs[x].frame,
                self.state.legs[x].point)
                for x in legs]

        return poly

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
