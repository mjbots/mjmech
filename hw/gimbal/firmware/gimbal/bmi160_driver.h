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

#include "async_types.h"
#include "imu_data.h"
#include "pool_ptr.h"

class AsyncI2C;
class Clock;
class PersistentConfig;
class TelemetryManager;

class Bmi160Driver {
 public:
  Bmi160Driver(Pool&, const gsl::cstring_span& name,
               AsyncI2C&, Clock&, PersistentConfig&, TelemetryManager&);
  ~Bmi160Driver();

  void AsyncStart(ErrorCallback);
  void Poll();

  ImuDataSignal* data_signal();

  struct Config {
    uint8_t address = 0xd0;
    uint16_t rate_hz = 100;
    uint16_t gyro_max_dps = 1000;
    uint8_t accel_max_g = 4;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(address));
      a->Visit(MJ_NVP(rate_hz));
      a->Visit(MJ_NVP(gyro_max_dps));
      a->Visit(MJ_NVP(accel_max_g));
    }
  };

  enum State {
    kInitial,
    kIdentifying,
    kPoweringAccel,
    kPoweringGyro,
    kConfiguring,
    kErrorCheck,
    kOperational,
    kFault,
    kNumStates,
  };

  static std::array<std::pair<State, const char*>, kNumStates> StateMapper() {
    return std::array<std::pair<State, const char*>, kNumStates> { {
      { kInitial, "kInitial" },
      { kIdentifying, "kIdentifying" },
      { kPoweringAccel, "kPoweringAccel" },
      { kPoweringGyro, "kPoweringGyro" },
      { kConfiguring, "kConfiguring" },
      { kErrorCheck, "kErrorCheck" },
      { kOperational, "kOperational" },
      { kFault, "kFault" }, }
          };
  }

  struct Bmi160Data {
    State state = kInitial;

    // The sensor time as reported by the device.  This is in units of
    // 39us.
    uint32_t sensor_time = 0;

    // These are the actual used values, which may be different than
    // those configured due to what the device supports.
    uint16_t gyro_max_dps = 0;
    uint16_t accel_max_g = 0;
    uint16_t rate_hz = 0;

    uint32_t i2c_errors = 0;
    int32_t i2c_last_error = 0;
    int32_t i2c_first_error = 0;

    ImuData imu;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_ENUM(state, StateMapper));
      a->Visit(MJ_NVP(sensor_time));
      a->Visit(MJ_NVP(gyro_max_dps));
      a->Visit(MJ_NVP(accel_max_g));
      a->Visit(MJ_NVP(rate_hz));
      a->Visit(MJ_NVP(i2c_errors));
      a->Visit(MJ_NVP(i2c_last_error));
      a->Visit(MJ_NVP(i2c_first_error));
      a->Visit(MJ_NVP(imu));
    }
  };

  const Bmi160Data* data() const;

 private:
  class Impl;
  PoolPtr<Impl> impl_;
};
