#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import pytest

import leg_ik

def test_lizard_3dof():
    config = leg_ik.Configuration()
    config.coxa_length_mm = 50
    config.femur_length_mm = 40
    config.tibia_length_mm = 30
    config.coxa_min_deg = -90
    config.coxa_max_deg = 90
    config.femur_min_deg = -90
    config.femur_max_deg = 90
    config.tibia_min_deg = -90
    config.tibia_max_deg = 90
    config.coxa_ident = 3
    config.femur_ident = 4
    config.tibia_ident = 5

    point = leg_ik.Point3D(0, 90, -30)
    result = leg_ik.lizard_3dof_ik(point, config)

    assert abs(result.coxa_deg) < 0.01
    assert abs(result.femur_deg) < 0.01
    assert abs(result.tibia_deg) < 0.01

    point = leg_ik.Point3D(0, 90, -25)
    result = leg_ik.lizard_3dof_ik(point, config)

    assert abs(result.coxa_deg) < 0.01
    assert abs(result.femur_deg - 7.18) < 0.01
    assert abs(result.tibia_deg + 6.58) < 0.01

    point = leg_ik.Point3D(0, 90, -35)
    result = leg_ik.lizard_3dof_ik(point, config)

    assert abs(result.coxa_deg) < 0.01
    assert abs(result.femur_deg + 7.18) < 0.01
    assert abs(result.tibia_deg - 7.78) < 0.01

    point = leg_ik.Point3D(0, 95, -30)
    result = leg_ik.lizard_3dof_ik(point, config)

    assert abs(result.coxa_deg) < 0.01
    assert abs(result.femur_deg + 0.60) < 0.01
    assert abs(result.tibia_deg - 10.20) < 0.01

    # Now test some with coxa.

    point = leg_ik.Point3D(20, 87.75, -30)
    result = leg_ik.lizard_3dof_ik(point, config)

    assert abs(result.coxa_deg - 12.84) < 0.01
    assert abs(result.femur_deg) < 0.01
    assert abs(result.tibia_deg) < 0.01

    point = leg_ik.Point3D(20, 87.75, -25)
    result = leg_ik.lizard_3dof_ik(point, config)

    assert abs(result.coxa_deg - 12.84) < 0.01
    assert abs(result.femur_deg - 7.18) < 0.01
    assert abs(result.tibia_deg + 6.58) < 0.01

    command_dict = result.command_dict()
    assert sorted(command_dict.keys()) == [3, 4, 5]
    assert command_dict[3] == result.coxa_deg
    assert command_dict[4] == result.femur_deg
    assert command_dict[5] == result.tibia_deg
