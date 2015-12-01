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

#include "static_signal.h"

struct ImuData {
  uint32_t timestamp = {};
  int32_t error = 0;
  float gyro_x_dps = {};
  float gyro_y_dps = {};
  float gyro_z_dps = {};
  float accel_x_g = {};
  float accel_y_g = {};
  float accel_z_g = {};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(error));
    a->Visit(MJ_NVP(gyro_x_dps));
    a->Visit(MJ_NVP(gyro_y_dps));
    a->Visit(MJ_NVP(gyro_z_dps));
    a->Visit(MJ_NVP(accel_x_g));
    a->Visit(MJ_NVP(accel_y_g));
    a->Visit(MJ_NVP(accel_z_g));
  }
};

typedef StaticSignal<void (const ImuData*)> ImuDataSignal;
