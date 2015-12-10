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
#include "pid.h"
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

  enum State {
    kInitializing,
    kOperating,
    kFault,
    kNumStates,
  };

  static std::array<std::pair<State, const char*>, kNumStates> StateMapper() {
    return { {
        { kInitializing, "kInitializing" },
        { kOperating, "kOperating" },
        { kFault, "kFault" },
      } };
  }

  struct Data {
    State state;
    PID::State pitch;
    PID::State yaw;

    uint32_t start_timestamp = 0;

    Euler desired_deg;
    Point3D desired_body_rate_dps;
    uint32_t last_ahrs_update = 0;
    bool torque_on = false;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_ENUM(state, StateMapper));
      a->Visit(MJ_NVP(pitch));
      a->Visit(MJ_NVP(yaw));
      a->Visit(MJ_NVP(start_timestamp));
      a->Visit(MJ_NVP(desired_deg));
      a->Visit(MJ_NVP(desired_body_rate_dps));
      a->Visit(MJ_NVP(last_ahrs_update));
      a->Visit(MJ_NVP(torque_on));
    }
  };

  const Data& data() const;

 private:
  class Impl;
  PoolPtr<Impl> impl_;
};
