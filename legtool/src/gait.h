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

#include "common.h"
#include "leg_ik.h"
#include "tf.h"
#include "visitor.h"

namespace legtool {
struct Leg {
  enum class Mode {
    kStance,
      kSwing,
      kUnknown,
  };

  struct Config {
    Point3D mount_mm;
    Point3D idle_mm;
    std::shared_ptr<IKSolver> leg_ik;

    template <typename Achive>
    void Serialize(Archive* a) {
      a->Visit(LT_NVP(mount_mm));
      a->Visit(LT_NVP(idle_mm));
    }
  };

  struct Result {
    Point3D point;
    Mode mode;
  };

  struct State {
    Point3D point;
    Mode mode = kUnknown;
    IKSolver* leg_ik = nullptr;
    Frame* frame = nullptr;
    Frame* shoulder_frame = nullptr;
  };
};

struct MechanicalConfig {
  std::vector<Leg::Config> leg_config;
  Point3D body_cog_mm;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(LT_NVP(leg_config));
    a->Visit(LT_NVP(body_cog_mm));
  }
};

struct Command {
  double translate_x_mm_s = 0;
  double translate_y_mm_s = 0;
  double rotate_deg_s = 0;
  double body_x_mm = 0;
  double body_y_mm = 0;
  double body_z_mm = 0;
  double body_pitch_deg = 0;
  double body_roll_deg = 0;
  double body_yaw_deg = 0;
  double lift_height_percent = 0;
};

struct CommonState {
  std::vector<Leg::State> legs;
  Frame world_frame;
  Frame robot_frame;

};
}
