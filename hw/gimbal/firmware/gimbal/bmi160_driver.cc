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

#include "bmi160_driver.h"

#include <cmath>

#include "async_i2c.h"
#include "clock.h"
#include "persistent_config.h"
#include "telemetry_manager.h"

namespace {
enum class BMI160 {
  CHIP_ID = 0x00,
    CHIP_ID_VALUE = 0xd1,
    ERR_REG = 0x02,
    PMU_STATUS = 0x03,
    PMU_acc_suspend = 0x00,
    PMU_acc_normal = 0x01,
    PMU_gyr_suspect = 0x00,
    PMU_gyr_normal = 0x01,
    DATA_0 = 0x04,
    DATA_MAG = 0x04,
    DATA_GYR = 0x0c,
    DATA_ACC = 0x12,
    SENSORTIME = 0x18,
    STATUS = 0x1b,
    TEMPERATURE = 0x20,
    ACC_CONF = 0x40,
    ACC_RANGE = 0x41,
    GYR_CONF = 0x42,
    GYR_RANGE = 0x43,
    SELF_TEXT = 0x6d,
    CMD = 0x7e,
    CMD_acc_set_pmu_mode = 0x10,
    CMD_gyr_set_pmu_mode = 0x14,
};

const auto bi = [](BMI160 val) { return static_cast<int>(val); };

uint8_t FindODR(uint16_t rate_hz) {
  // Let's do the dumb thing and just try every possible value,
  // picking the closest one that doesn't go over.
  for (uint8_t result = 2; result <= 13; result++) {
    float this_hz = 100.0f / std::pow(2.0f, (8.0f - result));
    if (this_hz > rate_hz) { return result - 1; }
  }
  // 3200 is the fastest that either device supports.
  return 13;
}

uint16_t CalculateRate(uint8_t value) {
  return static_cast<uint16_t>(100.0f / std::pow(2.0f, (8.0f - value)));
}

uint8_t FindAccRange(uint8_t accel_max_g) {
  if (accel_max_g <= 2) { return 0x03; }
  if (accel_max_g <= 4) { return 0x05; }
  if (accel_max_g <= 8) { return 0x08; }
  if (accel_max_g <= 16) { return 0x0c; }
  return 0x0c;
}

uint8_t CalculateAccRange(uint8_t value) {
  if (value == 0x03) { return 2; }
  if (value == 0x05) { return 4; }
  if (value == 0x08) { return 8; }
  if (value == 0x0c) { return 16; }
  return 0;
}

uint8_t FindGyrRange(uint16_t range_dps) {
  if (range_dps <= 125) { return 0x04; }
  else if (range_dps <= 250) { return 0x03; }
  else if (range_dps <= 500) { return 0x02; }
  else if (range_dps <= 1000) { return 0x01; }
  else { return 0x00; }
}

uint16_t CalculateGyrRange(uint8_t range) {
  if (range == 0) { return 2000; }
  else if (range == 1) { return 1000; }
  else if (range == 2) { return 500; }
  else if (range == 3) { return 250; }
  else if (range == 4) { return 125; }
  return 0;
}
}

class Bmi160Driver::Impl {
 public:
  Impl(const gsl::cstring_span& name,
       AsyncI2C& async_i2c,
       Clock& clock,
       PersistentConfig& config,
       TelemetryManager& telemetry)
      : async_i2c_(async_i2c), clock_(clock) {
    config.Register(name, &config_);
    data_updater_ = telemetry.Register(name, &data_);
  }

  void AsyncStart(ErrorCallback callback) {
    min_operational_delay_ = std::max(0, 10000 / config_.rate_hz - 1);
    start_callback_ = callback;
    ConfigCallback(0);
  }

  void Poll() {
    const ErrorCallback callback =
        [this](int error) { this->ConfigCallback(error); };

    switch (data_.state) {
      case kInitial:
      case kIdentifying:
      case kConfiguring:
      case kWaitingToPower: {
        break;
      }
      case kPoweringAccel: {
        const uint32_t timestamp = clock_.timestamp();
        const int32_t delta = timestamp - last_event_timestamp_;
        if (delta < 60) { return; }

        buffer_[0] = bi(BMI160::CMD_gyr_set_pmu_mode) |
                     bi(BMI160::PMU_gyr_normal);
        data_.state = kPoweringGyro;
        async_i2c_.AsyncWrite(
            config_.address,
            bi(BMI160::CMD),
            gsl::cstring_span(buffer_, 1),
            callback);
        break;
      }
      case kPoweringGyro: {
        const uint32_t timestamp = clock_.timestamp();
        const int32_t delta = timestamp - last_event_timestamp_;
        if (delta < 900) { return; }

        data_.state = kErrorCheck;
        async_i2c_.AsyncRead(
            config_.address,
            bi(BMI160::ERR_REG),
            gsl::string_span(buffer_, 1),
            callback);

        break;
      }
      case kErrorCheck:
      case kFault: {
        break;
      }
      case kOperational: {
        if (operational_busy_) { return; }
        const uint32_t timestamp = clock_.timestamp();
        const int32_t delta = timestamp - last_event_timestamp_;

        if (delta < min_operational_delay_) { return; }

        StartStatusRead();
        break;
      }
      case kNumStates: {
        assert(false);
      }
    }
  }

  void ConfigCallback(int error) {
    const ErrorCallback callback =
        [this](int error) { this->ConfigCallback(error); };

    if (error) {
      // For now, all configuration errors are fatal.
      data_.state = kFault;
      data_.imu.error = 0x10000 | error;
      Emit();
      start_callback_(data_.imu.error);
      return;
    }

    switch (data_.state) {
      case kInitial: {
        // Verify that the correct device is present.
        data_.state = kIdentifying;
        async_i2c_.AsyncRead(
            config_.address,
            bi(BMI160::CHIP_ID),
            gsl::string_span(buffer_, 1),
            callback);
        break;
      }
      case kIdentifying: {
        const uint8_t received_id = static_cast<uint8_t>(buffer_[0]);
        if (received_id != bi(BMI160::CHIP_ID_VALUE)) {
          data_.imu.error = 0x40000 | received_id;
          Emit();
          start_callback_(data_.imu.error);
          return;
        }

        // Now, configure the devices.
        const int kStart = bi(BMI160::ACC_CONF);

        const int acc_us = 0 << 7;
        const int acc_bwp = 2 << 4; // Only valid value for acc_us = 0;
        const int acc_odr = FindODR(config_.rate_hz) << 0;

        buffer_[bi(BMI160::ACC_CONF) - kStart] = acc_us | acc_bwp | acc_odr;

        const int acc_range = FindAccRange(config_.accel_max_g);
        buffer_[bi(BMI160::ACC_RANGE) - kStart] = acc_range;

        data_.rate_hz = CalculateRate(acc_odr);
        data_.accel_max_g = CalculateAccRange(acc_range);

        const int gyr_bwp = 0x02 << 4; // Normal mode.
        const int gyr_odr = FindODR(config_.rate_hz) << 0;
        buffer_[bi(BMI160::GYR_CONF) - kStart] = gyr_bwp | gyr_odr;

        const int gyr_range = FindGyrRange(config_.gyro_max_dps);
        buffer_[bi(BMI160::GYR_RANGE)] = gyr_range;

        data_.gyro_max_dps = CalculateGyrRange(gyr_range);

        data_.state = kConfiguring;
        async_i2c_.AsyncWrite(
            config_.address, kStart, gsl::cstring_span(buffer_, buffer_ + 4),
            callback);
        break;
      }
      case kConfiguring: {
        buffer_[0] = 0xff;
        data_.state = kWaitingToPower;
        // fall through
      }
      case kWaitingToPower: {
        if (buffer_[0] != 0x00) {
          async_i2c_.AsyncRead(
              config_.address,
              bi(BMI160::CMD),
              gsl::string_span(buffer_, 1),
              callback);
        } else {
          buffer_[0] = bi(BMI160::CMD_acc_set_pmu_mode) |
                       bi(BMI160::PMU_acc_normal);
          data_.state = kPoweringAccel;
          last_event_timestamp_ = clock_.timestamp();
          async_i2c_.AsyncWrite(
              config_.address,
              bi(BMI160::CMD),
              gsl::cstring_span(buffer_, 1),
              callback);
        }
        break;
      }
      case kPoweringAccel: { return; }
      case kPoweringGyro: { return; }
      case kErrorCheck: {
        if (buffer_[0] != 0) {
          data_.state = kFault;
          data_.imu.error = 0x20000 | static_cast<uint8_t>(buffer_[0]);
          start_callback_(data_.imu.error);
          return;
        }
        data_.state = kOperational;
        last_event_timestamp_ = clock_.timestamp();
        start_callback_(0);
        break;
      }
      case kOperational: // fall-through
      case kFault:
      case kNumStates: {
        assert(false);
        break;
      }
    }

    Emit();
  }

  void Emit() {
    data_.imu.timestamp = clock_.timestamp();
    data_updater_();
  }

  void StartStatusRead() {
    // We first hot spin looking for the data ready flag to be true.
    // Once that is the case, then we read the whole buffer.
    last_event_timestamp_ = clock_.timestamp();
    operational_busy_ = true;
    async_i2c_.AsyncRead(
        config_.address,
        static_cast<int>(BMI160::STATUS),
        gsl::string_span(buffer_, 1),
        [this](int error) { this->HandleStatusRead(error); });
  }

  void HandleStatusRead(int error) {
    if (error) {
      Fault(0x30000 | error);
      return;
    }

    const uint8_t status = static_cast<uint8_t>(buffer_[0]);
    const int drdy_acc = 1 << 7;
    const int drdy_gyr = 1 << 6;
    if ((status & drdy_acc) || (status & drdy_gyr)) {
      StartDataRead();
    } else {
      StartStatusRead();
    }
  }

  void StartDataRead() {
    async_i2c_.AsyncRead(
        config_.address,
        static_cast<int>(BMI160::DATA_GYR),
        gsl::string_span(buffer_),
        [this](int error) { this->HandleDataRead(error); });
  }

  void HandleDataRead(int error) {
    if (error) {
      Fault(0x40000 | error);
      return;
    }

    // Actually handle the data.
    HandleData();

    operational_busy_ = false;
  }

  void Fault(int error) {
    // For now, all faults are fatal.
    data_.imu.error = error;
    data_.state = kFault;
    Emit();
    data_signal_(&data_.imu);
  }

  void HandleData() {
    const int kStart = bi(BMI160::DATA_GYR);
    const auto time = [&](int position) -> int {
      return static_cast<uint8_t>(
          buffer_[bi(BMI160::SENSORTIME) - kStart + position]);
    };
    data_.sensor_time = time(0) << 0 | time(1) << 8 | time(2) << 16;

    const auto data = [&](int position) -> int {
      const uint16_t uval = (
          static_cast<uint16_t>(
              static_cast<uint8_t>(
                  buffer_[bi(BMI160::DATA_GYR) - kStart + position])) |
          static_cast<uint16_t>(
              static_cast<uint8_t>(
                  buffer_[bi(BMI160::DATA_GYR) -
                          kStart + position + 1])) << 8);
      return (uval > 32767) ? (uval - 65536) : uval;
    };

    const int gyr_x = data(0);
    const int gyr_y = data(2);
    const int gyr_z = data(4);
    const int acc_x = data(6);
    const int acc_y = data(8);
    const int acc_z = data(10);

    const float gyro_scale = FindGyroScale();
    data_.imu.gyro_x_dps = gyr_x * gyro_scale;
    data_.imu.gyro_y_dps = gyr_y * gyro_scale;
    data_.imu.gyro_z_dps = gyr_z * gyro_scale;

    const float accel_scale = FindAccelScale();
    data_.imu.accel_x_g = acc_x * accel_scale;
    data_.imu.accel_y_g = acc_y * accel_scale;
    data_.imu.accel_z_g = acc_z * accel_scale;

    Emit();
  }

  float FindGyroScale() const {
    const auto rate = data_.gyro_max_dps;
    if (rate == 2000) { return 1.0f / 16.4f; }
    else if (rate == 1000) { return 1.0f / 32.8f; }
    else if (rate == 500) { return 1.0f / 65.6f; }
    else if (rate == 250) { return 1.0f / 131.2f; }
    else if (rate == 125) { return 1.0f / 262.4f; }
    return 0.0;
  }

  float FindAccelScale() const {
    const auto rate = data_.accel_max_g;
    if (rate == 2) { return 1.0f / 16384.0f; }
    else if (rate == 4) { return 1.0f / 8192.0f; }
    else if (rate == 8) { return 1.0 / 4096.0f; }
    else if (rate == 16) { return 1.0 / 2048.0f; }
    return 0.0;
  }

  AsyncI2C& async_i2c_;
  const Clock& clock_;
  Config config_;
  Bmi160Data data_;
  ImuDataSignal data_signal_;
  StaticFunction<void ()> data_updater_;
  ErrorCallback start_callback_;
  uint32_t last_event_timestamp_ = 0;
  bool operational_busy_ = false;
  int32_t min_operational_delay_ = 0;

  enum {
    kBufferSize = (static_cast<int>(BMI160::STATUS) -
                   static_cast<int>(BMI160::DATA_GYR) + 1),
  };
  char buffer_[kBufferSize] = {};
};

Bmi160Driver::Bmi160Driver(Pool& pool,
                           const gsl::cstring_span& name,
                           AsyncI2C& async_i2c,
                           Clock& clock,
                           PersistentConfig& config,
                           TelemetryManager& telemetry)
  : impl_(&pool, name, async_i2c, clock, config, telemetry) {
}

Bmi160Driver::~Bmi160Driver() {}

void Bmi160Driver::AsyncStart(ErrorCallback callback) {
  impl_->AsyncStart(callback);
}

void Bmi160Driver::Poll() { impl_->Poll(); }

ImuDataSignal* Bmi160Driver::data_signal() { return &impl_->data_signal_; }

const Bmi160Driver::Bmi160Data* Bmi160Driver::data() const {
  return &impl_->data_;
}
