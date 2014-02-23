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

    point = leg_ik.Point3D(90, 0, 30)
    result = leg_ik.lizard_3dof_ik(point, config)

    assert abs(result.coxa_deg) < 0.01
    assert abs(result.femur_deg) < 0.01
    assert abs(result.tibia_deg) < 0.01

    point = leg_ik.Point3D(90, 0, 25)
    result = leg_ik.lizard_3dof_ik(point, config)

    assert abs(result.coxa_deg) < 0.01
    assert abs(result.femur_deg - 7.18) < 0.01
    assert abs(result.tibia_deg + 6.58) < 0.01

    point = leg_ik.Point3D(90, 0, 35)
    result = leg_ik.lizard_3dof_ik(point, config)

    assert abs(result.coxa_deg) < 0.01
    assert abs(result.femur_deg + 7.18) < 0.01
    assert abs(result.tibia_deg - 7.78) < 0.01

    point = leg_ik.Point3D(95, 0, 30)
    result = leg_ik.lizard_3dof_ik(point, config)

    assert abs(result.coxa_deg) < 0.01
    assert abs(result.femur_deg + 0.60) < 0.01
    assert abs(result.tibia_deg - 10.20) < 0.01

    # Now test some with coxa.

    point = leg_ik.Point3D(87.75, 20, 30)
    result = leg_ik.lizard_3dof_ik(point, config)

    assert abs(result.coxa_deg - 12.84) < 0.01
    assert abs(result.femur_deg) < 0.01
    assert abs(result.tibia_deg) < 0.01

    point = leg_ik.Point3D(87.75, 20, 25)
    result = leg_ik.lizard_3dof_ik(point, config)

    assert abs(result.coxa_deg - 12.84) < 0.01
    assert abs(result.femur_deg - 7.18) < 0.01
    assert abs(result.tibia_deg + 6.58) < 0.01
