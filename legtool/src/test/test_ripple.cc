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

#include <boost/format.hpp>
#include <boost/test/auto_unit_test.hpp>

namespace {
using namespace legtool;

void CheckPoints(const Point3D& lhs, const Point3D& rhs) {
  BOOST_CHECK_SMALL(lhs.x - rhs.x, 1e-2);
  BOOST_CHECK_SMALL(lhs.y - rhs.y, 1e-2);
  BOOST_CHECK_SMALL(lhs.z - rhs.z, 1e-2);
}

template <typename T>
void SanityCheckState(const T& state) {
  BOOST_CHECK_EQUAL(state.robot_frame.parent, &state.world_frame);
  BOOST_CHECK_EQUAL(state.body_frame.parent, &state.robot_frame);

  std::set<const Frame*> valid_frames{
    &state.world_frame, &state.robot_frame, &state.body_frame
        };

  int stance_legs = 0;
  for (const auto& leg: state.legs) {
    BOOST_CHECK_EQUAL(valid_frames.count(leg.frame), 1);

    if (leg.mode == Leg::Mode::kStance) {
      stance_legs += 1;
      BOOST_CHECK_EQUAL(leg.frame, &state.world_frame);
    } else if (leg.mode == Leg::Mode::kSwing) {
    } else {
      BOOST_CHECK_MESSAGE(false,
                          (boost::format("unknown leg mode %d") %
                           static_cast<int>(leg.mode)).str());
    }
  }

  // Require at least 2 legs in stance at all times.
  BOOST_CHECK_GE(stance_legs, 2);
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

template <typename Gait, typename Command>
void RunCycle(Gait& gait, const Command& command,
              double total_time_s, double time_step_s) {
  gait.SetCommand(command);

  double cur_time_s = 0.0;
  while (cur_time_s < total_time_s) {
    const auto& joints = gait.AdvanceTime(time_step_s);
    cur_time_s += time_step_s;

    BOOST_CHECK_GE(joints.joints.size(), 1);

    SanityCheckState(gait.state());
  }
}
}

BOOST_AUTO_TEST_CASE(TestRippleBasic) {
  RippleConfig config = MakeConfig();
  RippleGait gait(config);

  // We want to:
  //  1. get the idle state
  //  2. initialize ourselves to that
  //
  // then,
  //  a) command no motion... verify stepping in place
  //  b) command forward motion
  //  c) command rotation
  //  d) do all of the above with each of the other 6 degrees of
  //     freedom altered

  // Also, we will want to test changes in command, and coming to a
  // stop.
  const auto state = gait.state();

  CheckPoints(state.legs.at(0).point, Point3D(190, 90, 0));
  CheckPoints(state.legs.at(1).point, Point3D(190, -90, 0));
  CheckPoints(state.legs.at(2).point, Point3D(-190, -90, 0));
  CheckPoints(state.legs.at(3).point, Point3D(-190, 90, 0));

  SanityCheckState(state);

  Command command;
  RunCycle(gait, command, 10.0, 0.01);
}
