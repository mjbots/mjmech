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

struct Euler {
  float yaw = 0.0f;
  float pitch = 0.0f;
  float roll = 0.0f;

  Euler() {}
  Euler(float yaw, float pitch, float roll)
      : yaw(yaw), pitch(pitch), roll(roll) {}

  Euler scaled(float scale) const {
    return Euler{yaw * scale, pitch * scale, roll * scale};
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(yaw));
    a->Visit(MJ_NVP(pitch));
    a->Visit(MJ_NVP(roll));
  }
};
