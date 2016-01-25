// Copyright 2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

struct BldcEncoderData {
  uint32_t timestamp = 0;
  float position_deg = 0.0f;
  float phase = 0.0f;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(position_deg));
    a->Visit(MJ_NVP(phase));
  }
};

typedef StaticSignal<void (const BldcEncoderData*)> BldcEncoderDataSignal;
