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

#pragma once

#include "gait.h"

namespace legtool {
struct RippleConfig {
  MechanicalConfig mechanical;

  double max_cycle_time_s = 4.0;
  double lift_height_mm = 80.0;
  double lift_percent = 25.0;
  double swing_percent = 80.0;
  double position_margin_percent = 80.0;
  std::vector<std::vector<int>> leg_order;
  double body_z_offset_mm = 0.0;
  double servo_speed_margin_percent = 70.0;
  bool statically_stable = false;
  double static_center_factor = 3.0;
  double static_stable_factor = 10.0;
  double static_margin_mm = 20.0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(LT_NVP(mechanical));
    a->Visit(LT_NVP(max_cycle_time_s));
    a->Visit(LT_NVP(lift_height_mm));
    a->Visit(LT_NVP(lift_percent));
    a->Visit(LT_NVP(swing_percent));
    a->Visit(LT_NVP(position_margin_percent));
    a->Visit(LT_NVP(leg_order));
    a->Visit(LT_NVP(body_z_offset_mm));
    a->Visit(LT_NVP(servo_speed_margin_percent));
    a->Visit(LT_NVP(statically_stable));
    a->Visit(LT_NVP(static_center_factor));
    a->Visit(LT_NVP(static_stable_factor));
    a->Visit(LT_NVP(static_margin_mm));
  }
};

struct RippleState : public CommonState {
  double phase = 0.0;
  enum Action {
    kActionStartSwing,
    kActionStartStance,
    kActionEnd,
  } action = kActionStartSwing;

  struct Leg : public legtool::Leg::State {
    boost::optional<Point3D> swing_start_pos;
    boost::optional<Point3D> swing_end_pos;
  };

  std::vector<Leg> legs;

  RippleState() {}
  RippleState(const RippleState& rhs) : CommonState(rhs) {
    legs = rhs.legs;

    auto map_frame = [this, &rhs](const Frame* frame) {
      if (frame == &rhs.world_frame) {
        return &world_frame;
      } else if (frame == &rhs.robot_frame) {
        return &robot_frame;
      } else if (frame == &rhs.body_frame) {
        return &body_frame;
      } else if (frame == &rhs.cog_frame) {
        return &cog_frame;
      } else if (frame == nullptr) {
        return static_cast<Frame*>(nullptr);
      }
      BOOST_ASSERT(false);
    };

    for (auto& leg: legs) {
      leg.frame = map_frame(leg.frame);
      leg.shoulder_frame = map_frame(leg.shoulder_frame);
    }
  }
};

struct Options {
  double cycle_time_s = 0.0;
  double servo_speed_dps = 0.0;
};

class RippleGait : public Gait {
 public:
  RippleGait(const RippleConfig& config)
    : config_(config),
      idle_state_(GetIdleState()),
      state_(idle_state_) {
  }

  virtual ~RippleGait() {}

  virtual JointCommand AdvancePhase(double delta_phase) {
    JointCommand result;
    return result;
  }

  virtual JointCommand AdvanceTime(double delta_s) {
    return AdvancePhase(delta_s / phase_time_s());
  }

  const RippleState& state() const { return state_; }

 private:
  double phase_time_s() const { return options_.cycle_time_s; }

  RippleState GetIdleState() const {
    RippleState result;

    result.body_frame.transform.translation.z = config_.body_z_offset_mm;

    for (const auto& leg_config: config_.mechanical.leg_config) {
      Point3D point;

      const double x_sign = GetSign(leg_config.mount_mm.x);
      point.x = leg_config.mount_mm.x + leg_config.idle_mm.x * x_sign;

      const double y_sign = GetSign(leg_config.mount_mm.y);
      point.y = leg_config.mount_mm.y + leg_config.idle_mm.y * y_sign;

      point.z = leg_config.mount_mm.z +
          leg_config.idle_mm.z -
          config_.body_z_offset_mm;

      RippleState::Leg leg_state;
      leg_state.point = result.world_frame.MapFromFrame(
          &result.body_frame, point);
      leg_state.frame = &result.world_frame;

      result.legs.push_back(leg_state);
    }

    return result;
  }

  const RippleConfig config_;
  Options options_;
  const RippleState idle_state_;

  RippleState state_;
};
}
