# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import pytest

import leg_ik
import ripple_gait
from ripple_gait import (STANCE, SWING)

def contains(sequence, value):
    return len([x for x in sequence if x is value]) == 1

def check_vectors_close(v1, v2):
    assert abs(v1.x - v2.x) < 0.001
    assert abs(v1.y - v2.y) < 0.001
    assert abs(v1.z - v2.z) < 0.001

def sanity_check_state(state):
    # Verify frames are properly linked.
    assert state.world_frame is not None
    assert state.robot_frame.parent is state.world_frame
    assert state.body_frame.parent is state.robot_frame

    stance_legs = 0
    for leg_num, leg in state.legs.iteritems():
        #  All legs are referenced to one of the valid frames.
        assert contains(
            [state.world_frame, state.robot_frame, state.body_frame],
            leg.frame)

        if leg.mode == STANCE:
            stance_legs += 1
            # Legs in stance are referenced relative to the world frame.
            assert leg.frame is state.world_frame
        elif leg.mode == SWING:
            pass
        else:
            assert False, "unknown leg mode: %d" % leg.mode

    #  At least 3 legs are in stance at all times.
    assert stance_legs >= 3

def run_cycle(gait, start_state, command, total_time_s, time_step_s):
    old_state = gait.set_state(start_state, command).copy()

    # Things we will want to verify.
    #
    #  * The robot moves at the expected rate/rotation

    cur_time_s = 0.0
    while cur_time_s < total_time_s:
        this_state = gait.advance_time(time_step_s)
        cur_time_s += time_step_s

        sanity_check_state(this_state)

        for leg_num in this_state.legs.keys():

            this_leg = this_state.legs[leg_num]
            old_leg = old_state.legs[leg_num]

            current_world_point = this_state.world_frame.map_from_frame(
                this_leg.frame, this_leg.point)
            old_world_point = old_state.world_frame.map_from_frame(
                old_leg.frame, old_leg.point)

            #  Legs identified as in stance do not move in the world frame.
            if this_leg.mode == STANCE and old_leg.mode == STANCE:
                check_vectors_close(current_world_point, old_world_point)

            # No leg moves faster than X in the world frame.
            world_speed_mm_s = (current_world_point -
                                old_world_point).length() / time_step_s
            assert world_speed_mm_s < 1000.0

            # No Leg moves faster than Y in the body frame.
            current_body_point = this_state.body_frame.map_from_frame(
                this_leg.frame, this_leg.point)
            old_body_point = old_state.body_frame.map_from_frame(
                old_leg.frame, old_leg.point)

            body_speed_mm_s = (current_body_point -
                               old_body_point).length() / time_step_s
            assert body_speed_mm_s < 500.0

        old_state = this_state.copy()

def test_ripple_basic():
    config = ripple_gait.RippleConfig()

    mounts = [(90., 90.), (90., -90.), (-90., -90.), (-90., 90.)]

    ik_config = leg_ik.Configuration()
    ik_config.coxa_min_deg = -120.0
    ik_config.coxa_max_deg = 120.0
    ik_config.coxa_length_mm = 60.0
    ik_config.femur_min_deg = -120.0
    ik_config.femur_max_deg = 120.0
    ik_config.femur_length_mm = 60.0
    ik_config.tibia_min_deg = -120.0
    ik_config.tibia_max_deg = 120.0
    ik_config.tibia_length_mm = 60.0

    for leg_num in range(4):
        leg = ripple_gait.LegConfig()

        leg.mount_x_mm = mounts[leg_num][0]
        leg.mount_y_mm = mounts[leg_num][1]
        leg.mount_z_mm = 0.0

        leg.idle_x_mm = 100.0
        leg.idle_y_mm = 0.0
        leg.idle_z_mm = 0.0

        leg.leg_ik = leg_ik.LizardIk(ik_config)

        config.mechanical.leg_config[leg_num] = leg

    config.mechanical.body_cog_x_mm = 0.0
    config.mechanical.body_cog_y_mm = 0.0
    config.mechanical.body_cog_z_mm = 0.0

    config.max_cycle_time_s = 4.0
    config.lift_height_mm = 20.0
    config.min_swing_percent = 50.0
    config.max_swing_percent = 100.0
    config.leg_order = [0, 2, 1, 3]
    config.body_z_offset_mm = 60.0

    gait = ripple_gait.RippleGait(config)

    # We want to:
    #  1. get the idle state
    #  2. initialize ourselves to that
    #
    # then,
    #  a) command no motion... verify stepping in place
    #  b) command forward motion
    #  c) command rotation
    #  d) do all of the above with each of the other 6 degrees of
    #     freedom altered

    # Also, we will want to test changes in command, and coming to a
    # stop.

    idle_state = gait.get_idle_state()

    assert len(idle_state.legs) == 4
    assert idle_state.legs[0].point == leg_ik.Point3D(190, 90, 0)
    assert idle_state.legs[1].point == leg_ik.Point3D(190, -90, 0)
    assert idle_state.legs[2].point == leg_ik.Point3D(-190, -90, 0)
    assert idle_state.legs[3].point == leg_ik.Point3D(-190, 90, 0)

    gait_graph = gait.get_gait_graph()
    assert len(gait_graph.leg) == 4

    sanity_check_state(idle_state)

    command = ripple_gait.Command()
    run_cycle(gait, idle_state, command, 10.0, 0.01)
    run_cycle(gait, idle_state, command, 10.0, 0.0073)

    config.max_cycle_time_s = 1.7
    gait = ripple_gait.RippleGait(config)
    run_cycle(gait, idle_state, command, 5.0, 0.01)
