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

#include <cstdint>

/// This class lets you set the PWM values for the three phases of a
/// brushless motor.
class BldcPwm {
 public:
  BldcPwm() {}
  virtual ~BldcPwm() {}
  BldcPwm(const BldcPwm&) = delete;

  /// PWM values logically span the entire 16 bit range.  0 should be
  /// fully off, and 65535 should be fully on.
  virtual void Set(uint16_t phase_a,
                   uint16_t phase_b,
                   uint16_t phase_c) = 0;

  // TODO jpieper: Probably need ability to read fault flags.
};
