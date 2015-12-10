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

#include "ahrs_data.h"
#include "async_types.h"
#include "pool_ptr.h"

class BldcPwm;
class Clock;
class GpioPin;
class PersistentConfig;
class TelemetryManager;

class GimbalStabilizer {
 public:
  GimbalStabilizer(Pool&, Clock&, PersistentConfig&,
                   TelemetryManager&, AhrsDataSignal&,
                   GpioPin& motor_enable,
                   BldcPwm& motor1, BldcPwm& motor2);
  ~GimbalStabilizer();

  /// Clear all fault states and start again.
  void Reset();

  /// When set true and not in a fault state, power is applied to the
  /// motors.
  void SetTorque(bool);

  /// Attempt to stabilize the IMU at the given pitch and yaw in the
  /// AHRS reference frame.
  void SetImuAttitude(float pitch_deg, float yaw_deg);

  /// Set an equivalent IMU yaw command so as to position the unit at
  /// the given absolute yaw angle.
  void SetAbsoluteYaw(float yaw_deg);

  void Command(const gsl::cstring_span&, ErrorCallback);

  void PollMillisecond();

 private:
  class Impl;
  PoolPtr<Impl> impl_;
};
