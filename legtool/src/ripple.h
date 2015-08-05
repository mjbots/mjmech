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

// TODO jpieper
//
//  * During the swing phase, this forces the end effector to travel
//    in a robot-frame straight line to the new position.  All we
//    really care is that the Z position follows a given trajectory.
//    There could be alternate, faster or more efficient paths to
//    reach the new position.
//
//  * If the command is changed during a swing, the leg target
//    position is not updated.  The current representation doesn't
//    necessarily make that super easy, but it should be doable.
//    Right now, commands are only updated at the end of a full phase,
//    which is kind of a long time.
//
//  * I'm not sure how many heap operations are required for a single
//    gait update.  Ideally, it would be 0.

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
  };

  int action = 0;

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
      actions_(GetActionList()),
      state_(idle_state_) {
  }

  virtual ~RippleGait() {}

  virtual JointCommand AdvancePhase(double delta_phase) {
    if (config_.leg_order.empty()) {
      state_.phase = std::fmod(state_.phase + delta_phase, 1.0);
      return MakeJointCommand();
    }

    double cur_phase = state_.phase;
    double next_phase = state_.phase + delta_phase;

    for (;;) {
      const auto& next_action = actions_.at(state_.action);

      if (next_action.phase > next_phase) { break; }

      const double advance_phase = next_action.phase;
      if (advance_phase != cur_phase) {
        const double delta_phase = advance_phase - cur_phase;
        AdvancePhaseNoaction(delta_phase, advance_phase);
      }

      cur_phase = advance_phase;
      DoAction(state_.action);

      state_.action++;

      if (state_.action >= static_cast<int>(actions_.size())) {
        state_.action = 0;
        next_phase -= 1.0;
        cur_phase -= 1.0;

        // Install our new command.
      }
    }

    // Finally, advance the remainder of the phase and update our state.
    AdvancePhaseNoaction(next_phase - cur_phase, next_phase);
    state_.phase = next_phase;

    return MakeJointCommand();
  }

  virtual JointCommand AdvanceTime(double delta_s) {
    return AdvancePhase(delta_s / phase_time_s());
  }

  const RippleState& state() const { return state_; }

 private:
  struct Action;

  JointCommand MakeJointCommand() const {
    JointCommand result;
    for (const auto& leg: state_.legs) {
      Point3D shoulder_point =
          leg.shoulder_frame->MapFromFrame(leg.frame, leg.point);
      auto joints = leg.leg_ik->Solve(shoulder_point);
      for (const auto& joint: joints.joints) {
        result.joints.emplace_back(
            JointCommand::Joint(joint.ident, joint.angle_deg));
      }
    }

    return result;
  }

  void AdvancePhaseNoaction(double delta_phase, double final_phase) {
    const double dt = delta_phase * phase_time_s();

    const auto update_frame = GetUpdateFrame(dt);

    auto new_transform = update_frame.TransformToFrame(&state_.world_frame);
    state_.robot_frame.transform = new_transform;

    // Update the legs which are in swing.
    for (auto& leg: state_.legs) {
      if (leg.mode == Leg::Mode::kSwing) {
        double leg_phase =
            (std::fmod(final_phase, (1.0 / config_.leg_order.size())) /
             swing_phase_time());
        // Don't allow the phase to wrap-around on the final update.
        if (delta_phase > 0.0 && leg_phase == 0.0) {
          leg_phase = 1.0;
        }

        BOOST_ASSERT(leg.frame == &state_.robot_frame);
        const auto delta = *leg.swing_end_pos - *leg.swing_start_pos;
        const auto current = *leg.swing_start_pos + delta.scaled(leg_phase);
        leg.point = current;

        const double lift_fraction = 0.01 * config_.lift_percent;
        const double height_mm =
            config_.lift_height_mm *
            command_.lift_height_percent / 100.0;
        if (leg_phase < lift_fraction) {
          leg.point.z = (leg_phase / lift_fraction) * height_mm;
        } else if (leg_phase < (1.0 - lift_fraction)) {
          leg.point.z = height_mm;
        } else {
          leg.point.z = ((1.0 - leg_phase) / lift_fraction) * height_mm;
        }
      }
    }

    // TODO jpieper: Statically stable gait.
  }

  void DoAction(int action_index) {
    const auto& action = actions_[action_index];

    for (int leg_num: config_.leg_order[action.leg_group]) {
      auto& leg = state_.legs.at(leg_num);

      if (action.action == RippleState::kActionStartStance) {
        leg.mode = Leg::Mode::kStance;
        leg.point = state_.world_frame.MapFromFrame(
            leg.frame, leg.point);
        leg.point.z = 0;
        leg.frame = &state_.world_frame;
      } else if (action.action == RippleState::kActionStartSwing) {
        leg.mode = Leg::Mode::kSwing;
        leg.point = state_.robot_frame.MapFromFrame(
            leg.frame, leg.point);
        leg.frame = &state_.robot_frame;
        leg.swing_start_pos = leg.point;
        leg.swing_end_pos = GetSwingEndPos(leg_num);
      }
    }
  }

  Frame GetUpdateFrame(double dt) const {
    Frame result;
    result.parent = &state_.robot_frame;

    const double vx = command_.translate_x_mm_s;
    const double vy = command_.translate_y_mm_s;
    double dx = 0.0;
    double dy = 0.0;
    if (command_.rotate_deg_s == 0.0) {
      dx = vx * dt;
      dy = vy * dt;
    } else {
      const double vyaw = Radians(command_.rotate_deg_s);
      dx = ((std::cos(dt * vyaw) - 1) * vy +
            std::sin(dt * vyaw) * vx) / vyaw;
      dy = ((std::cos(dt * vyaw) - 1) * vx +
             std::sin(dt * vyaw) * vy) / vyaw;
      result.transform.rotation =
          Quaternion::FromEuler(0, 0, dt * vyaw);
    }

    result.transform.translation.x = dx;
    result.transform.translation.y = dy;

    return result;
  }

  Point3D GetSwingEndPos(int leg_num) const {
    // Target swing end positions such that during stance, the leg
    // will spend half its travel time reaching the idle position, and
    // half its travel time going beyond the idle position.

    const double stance_phase_time = 1.0 - swing_phase_time();
    const double dt = 0.5 * stance_phase_time * phase_time_s();

    Frame end_frame = GetUpdateFrame(dt);

    // TODO jpieper: This should map from whatever frame the idle
    // state leg was actually in.
    return end_frame.MapToParent(idle_state_.legs.at(leg_num).point);
  }

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

  std::vector<Action> GetActionList() const {
    std::vector<Action> result;

    if (config_.leg_order.empty()) { return result; }

    const double swing_time = swing_phase_time();

    for (int i = 0; i < static_cast<int>(config_.leg_order.size()); i++) {
      const double fraction =
          static_cast<double>(i) / config_.leg_order.size();
      Action action = { fraction, i, RippleState::kActionStartSwing };
      result.push_back(action);
      Action action2 = { fraction + swing_time, i,
                         RippleState::kActionStartStance };
      result.push_back(action2);
    }

    Action action3 = { 1.0, -1, RippleState::kActionEnd };
    result.push_back(action3);
    return result;
  }

  double swing_phase_time() const {
    return (1.0 / config_.leg_order.size()) * 0.01 * config_.swing_percent;
  }

  const RippleConfig config_;
  Options options_;
  const RippleState idle_state_;

  struct Action {
    double phase = 0.0;
    int leg_group = -1;
    RippleState::Action action;

    Action() {}
    Action(double phase, int leg_group, RippleState::Action action)
        : phase(phase), leg_group(leg_group), action(action) {}
  };

  const std::vector<Action> actions_;

  // Mutable state.

  RippleState state_;
  Command command_;
};
}
