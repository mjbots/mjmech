# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import pytest

import leg_ik
import ripple_gait

def test_ripple_basic():
    config = ripple_gait.RippleConfig()

    mounts = [(90., 90.), (90., -90.), (-90., -90.), (-90., 90.)]

    for leg_num in range(4):
        leg = ripple_gait.LegConfig()

        leg.mount_x_mm = mounts[leg_num][0]
        leg.mount_y_mm = mounts[leg_num][1]
        leg.mount_z_mm = 0.0

        leg.idle_x_mm = 100.0
        leg.idle_y_mm = 0.0
        leg.idle_z_mm = 0.0

        config.mechanical.leg_config[leg_num] = leg

    config.mechanical.body_cog_x_mm = 0.0
    config.mechanical.body_cog_y_mm = 0.0
    config.mechanical.body_cog_z_mm = 0.0

    config.max_cycle_time_s = 4.0
    config.light_height_mm = 20.0
    config.min_swing_percent = 50.0
    config.max_swing_percent = 100.0
    config.leg_order = [0, 2, 1, 3]

    gait = ripple_gait.RippleGait(config)

    # We want to:
    #  1. get the idle state
    #  2. initialize ourselves to that
    #
    # then,
    #  a) command no motion... should we stand still or step in place?
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

    # Things we will want to verify.
    #
    #  1. There is no slippage between legs identified as being in stance.
    #  2. The body moves at the expected rate/rotation
    #  3. At least 3 legs are in stance at all times.
