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

import pytest

from . import leg_ik

def get_lizard_config():
    config = leg_ik.Configuration()
    config.coxa_length_mm = 50
    config.femur_length_mm = 40
    config.tibia_length_mm = 30
    config.coxa_min_deg = -90
    config.coxa_idle_deg = 0
    config.coxa_max_deg = 90
    config.coxa_mass_kg = 0.06
    config.femur_min_deg = -90
    config.femur_idle_deg = 0
    config.femur_max_deg = 90
    config.femur_mass_kg = 0.07
    config.tibia_min_deg = -90
    config.tibia_idle_deg = 0
    config.tibia_max_deg = 90
    config.tibia_mass_kg = 0.08
    config.coxa_ident = 3
    config.femur_ident = 4
    config.tibia_ident = 5

    return config


def test_lizard_3dof():
    config = get_lizard_config()

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

    # Try adding in some idle to the coxa.
    config.coxa_idle_deg = 3.0
    result = leg_ik.lizard_3dof_ik(point, config)
    assert abs(result.coxa_deg - 15.84) < 0.01
    assert abs(result.femur_deg - 7.18) < 0.01
    assert abs(result.tibia_deg + 6.58) < 0.01

    # And some idle to femur.
    config.femur_idle_deg = 4.0
    result = leg_ik.lizard_3dof_ik(point, config)
    assert abs(result.coxa_deg - 15.84) < 0.01
    assert abs(result.femur_deg - 11.18) < 0.01
    assert abs(result.tibia_deg + 6.58) < 0.01

    # And some idle to tibia.
    config.tibia_idle_deg = 5.0
    result = leg_ik.lizard_3dof_ik(point, config)
    assert abs(result.coxa_deg - 15.84) < 0.01
    assert abs(result.femur_deg - 11.18) < 0.01
    assert abs(result.tibia_deg + 1.58) < 0.01

    # Now try setting the max coxa low enough that we should get None.
    config.coxa_max_deg = 15.0

    result = leg_ik.lizard_3dof_ik(point, config)
    assert result is None

    config.coxa_max_deg = 90.0
    result = leg_ik.lizard_3dof_ik(point, config)
    assert result is not None

    # And set the tibia max deg low enough to get None.
    config.femur_max_deg = 10.0
    result = leg_ik.lizard_3dof_ik(point, config)
    assert result is None

    # We'll assume the other bounds (min, and tibia) are correct for
    # now.


def test_lizard_3dof_cog():
    # Test some COG measurements.
    ik = leg_ik.LizardIk(get_lizard_config())

    point = leg_ik.Point3D(0, 90, -30)
    cog, mass = ik.cog_mass(point)

    assert abs(mass - 0.21) < 0.01
    assert abs(cog.x - 0.0) < 0.01
    assert abs(cog.y - 64.76) < 0.01
    assert abs(cog.z + 5.71) < 0.01

    point = leg_ik.Point3D(15, 90, -40)
    cog, mass = ik.cog_mass(point)

    assert abs(mass - 0.21) < 0.01
    assert abs(cog.x - 10.17) < 0.01
    assert abs(cog.y - 63.96) < 0.01
    assert abs(cog.z + 5.69) < 0.01
