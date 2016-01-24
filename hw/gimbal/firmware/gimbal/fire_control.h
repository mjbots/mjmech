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

#include "command_manager.h"
#include "pool_ptr.h"

class Clock;
class GpioPin;
class PersistentConfig;
class PwmPin;
class TelemetryManager;

class FireControl {
 public:
  FireControl(Pool&, Clock&, PersistentConfig&, TelemetryManager&,
              GpioPin& laser_enable,
              GpioPin& pwm_enable,
              PwmPin& aeg_pwm,
              PwmPin& agitator_pwm,
              GpioPin& arm_switch,
              GpioPin& arm_led);
  ~FireControl();

  void SetLaser(bool);
  void SetFire(uint8_t pwm, uint8_t time_100ms);
  void SetAgitator(uint8_t pwm);

  bool laser() const;
  uint8_t fire_pwm() const;
  uint8_t fire_time_100ms() const;
  uint8_t agitator_pwm() const;

  void Command(const gsl::cstring_span&, const CommandManager::Response&);

  void PollMillisecond();

 private:
  class Impl;
  PoolPtr<Impl> impl_;
};
