// Copyright 2014-2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ripple.h"

#include <boost/test/auto_unit_test.hpp>

namespace {
using namespace legtool;

void CheckPoints(const Point3D& lhs, const Point3D& rhs) {
  BOOST_CHECK_SMALL(lhs.x - rhs.x, 1e-2);
  BOOST_CHECK_SMALL(lhs.y - rhs.y, 1e-2);
  BOOST_CHECK_SMALL(lhs.z - rhs.z, 1e-2);
}

std::shared_ptr<IKSolver> MakeIKSolver(int start_servo_num) {
  LizardIK::Config config;
  config.coxa.min_deg = -120.0;
  config.coxa.idle_deg = 0.0;
  config.coxa.max_deg = 120.0;
  config.coxa.length_mm = 60.0;
  config.coxa.ident = start_servo_num + 0;

  config.femur.min_deg = -120.0;
  config.femur.idle_deg = 0.0;
  config.femur.max_deg = 120.0;
  config.femur.length_mm = 60.0;
  config.femur.ident = start_servo_num + 1;

  config.tibia.min_deg = -120.0;
  config.tibia.idle_deg = 0.0;
  config.tibia.max_deg = 120.0;
  config.tibia.length_mm = 60.0;
  config.tibia.ident = start_servo_num + 2;

  config.servo_speed_dps = 360.0;

  return std::shared_ptr<IKSolver>(new LizardIK(config));
}

RippleConfig MakeConfig() {
  RippleConfig result;

  double mounts[][2]= {
    { 90, 90 },
    { 90, -90 },
    { -90, -90 },
    { -90, 90 },
  };

  for (int i = 0; i < 4; i++) {
    Leg::Config leg_config;
    leg_config.mount_mm.x = mounts[i][0];
    leg_config.mount_mm.y = mounts[i][1];
    leg_config.mount_mm.z = 0;

    leg_config.idle_mm.x = 100.0;
    leg_config.idle_mm.y = 0;
    leg_config.idle_mm.z = 0;

    leg_config.leg_ik = MakeIKSolver(i * 3);

    result.mechanical.leg_config.push_back(leg_config);
  }

  result.mechanical.body_cog_mm.x = 0;
  result.mechanical.body_cog_mm.y = 0;
  result.mechanical.body_cog_mm.z = 0;

  result.max_cycle_time_s = 4.0;
  result.lift_height_mm = 20.0;
  result.swing_percent = 50.0;
  result.leg_order = { {0}, {2}, {1}, {3} };
  result.body_z_offset_mm = 60.0;

  return result;
}
}

BOOST_AUTO_TEST_CASE(TestRippleBasic) {
  RippleConfig config = MakeConfig();
  RippleGait gait(config);

  const auto& state = gait.state();

  CheckPoints(state.legs.at(0).point, Point3D(190, 90, 0));
  CheckPoints(state.legs.at(1).point, Point3D(190, -90, 0));
  CheckPoints(state.legs.at(2).point, Point3D(-190, -90, 0));
  CheckPoints(state.legs.at(3).point, Point3D(-190, 90, 0));
}
