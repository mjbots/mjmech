// Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "base/visitor.h"

#include "quaternion.h"
#include "static_signal.h"

struct AhrsData {
  uint32_t timestamp = {};
  int32_t error = 0;

  Quaternion attitude;

  float yaw_deg = 0.0f;
  float pitch_deg = 0.0f;
  float roll_deg = 0.0f;

  Point3D body_rate_dps;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(error));
    a->Visit(MJ_NVP(attitude));
    a->Visit(MJ_NVP(yaw_deg));
    a->Visit(MJ_NVP(pitch_deg));
    a->Visit(MJ_NVP(roll_deg));
    a->Visit(MJ_NVP(body_rate_dps));
  }
};

typedef StaticSignal<void (const AhrsData*)> AhrsDataSignal;
