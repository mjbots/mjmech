# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import pytest

import ripple_gait

def test_ripple_basic():
    config = ripple_gait.RippleConfig()

    mounts = [(90., 90.), (90., -90.), (-90., -90.), (-90., 90.)]

    for leg_num in range(4):
        leg = ripple_gait.LegConfig()
        leg.number = leg_num

        leg.mount_x_mm = mounts[leg_num][0]
        leg.mount_y_mm = mounts[leg_num][1]
        leg.mount_z_mm = 0.0

        leg.idle_x_mm = 100.0
        leg.idle_y_mm = 0.0
        leg.idle_z_mm = -60.0

        config.mechanical.leg_config.append(leg)

    config.mechanical.body_cog_x_mm = 0.0
    config.mechanical.body_cog_y_mm = 0.0
    config.mechanical.body_cog_z_mm = 0.0

    config.max_cycle_time_s = 4.0
    config.light_height_mm = 20.0
    config.min_swing_percent = 50.0
    config.max_swing_percent = 100.0
    config.leg_order = [0, 2, 1, 3]

    gait = ripple_gait.RippleGait(config)
