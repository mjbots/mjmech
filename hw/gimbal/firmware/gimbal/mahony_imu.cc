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

#include "mahony_imu.h"

#include "clock.h"
#include "persistent_config.h"
#include "telemetry_manager.h"

/// @file this implementation was inspired by Seb Madgwick's IMU
/// algorithms, although are not directly copied from any of them.

namespace {
struct Config {
  float bias_period_s = 1.0;
  float kp = 2.0;
  float ki = 0.1;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(bias_period_s));
    a->Visit(MJ_NVP(kp));
    a->Visit(MJ_NVP(ki));
  }
};
}

class MahonyImu::Impl {
 public:
  Impl(Clock& clock,
       PersistentConfig& config, TelemetryManager& telemetry,
       ImuDataSignal& imu_signal)
      : clock_(clock) {
    data_updater_ = telemetry.Register(gsl::ensure_z("ahrs"), &data_);
    imu_signal.Connect([this](const ImuData* data) { this->HandleImu(data); });

    config.Register(gsl::ensure_z("mahony"), &config_, [](){});
  }

  void HandleImu(const ImuData* data) {
    switch (data_.state) {
      case kInitialBias: { DoInitialBias(data); break; }
      case kOperating: { DoOperating(data); break; }
      case kNumStates: { assert(false); }
    }

    data_.ahrs.timestamp = clock_.timestamp();
    data_updater_();
  }

  void DoInitialBias(const ImuData* data) {
    const auto now = clock_.timestamp();

    if (data_.start_timestamp == 0) {
      data_.start_timestamp = now;
      return;
    }

    data_.bias_count++;

    const float t = data_.bias_count;
    data_.integral_dps =
        data_.integral_dps.scaled((t - 1) / t) +
        data->gyro_dps.scaled(-1.0 / t);

    const uint32_t delta = now - data_.start_timestamp;
    const float delta_s = static_cast<float>(delta) / clock_.ticks_per_second();

    if (delta_s >= config_.bias_period_s) {
      // Initialize our attitude with the current accelerometer
      // reading.
      const auto& a = data->accel_g;

      const float roll_rad = std::atan2(-a.x, a.z);
      const float pitch_rad =
          std::atan2(a.y, std::sqrt(a.x * a.x + a.z * a.z));

      Euler euler_rad;
      euler_rad.roll = roll_rad;
      euler_rad.pitch = pitch_rad;
      data_.ahrs.attitude = Quaternion::FromEuler(euler_rad);

      data_.bias_count = 0;
      data_.start_timestamp = now;

      data_.state = kOperating;
    }
  }

  void DoOperating(const ImuData* data) {
    const auto now = clock_.timestamp();

    const float kDegToRad = mjmech::base::kPi / 180.0;
    const float kRadToDeg = 1.0f / kDegToRad;

    const Point3D a_g = data->accel_g;
    const Point3D a = a_g.scaled(1.0 / a_g.length());

    const Point3D g_dps = data->gyro_dps;

    // Start initializing our output structure.
    auto& o = data_.ahrs;
    if (data->rate_hz != o.rate_hz) {
      // If the rate changed, then we need to start re-measuring our
      // actual rate.
      o.rate_hz = data->rate_hz;
      data_.bias_count = 0;
      data_.start_timestamp = now;
    }

    // Estimate the direction of gravity.
    Point3D rotated = o.attitude.conjugated().
                      Rotate(Point3D(0.0, 0.0, 0.5));

    // Then, the error between that estimate, and what we actually
    // saw.
    const Point3D err_dps = a.cross(rotated).scaled(kRadToDeg);

    // Determine integrative terms.
    data_.integral_dps += err_dps.scaled(config_.ki / data->rate_hz);

    // Apply the corrections.
    const Point3D cg_dps = g_dps + err_dps.scaled(config_.kp) + data_.integral_dps;

    // Now, multiply our existing quaternion with the integrated rate
    // of change.
    o.attitude =
        o.attitude *
        Quaternion::IntegrateRotationRate(cg_dps.scaled(kDegToRad),
                                          1.0f / data->rate_hz);

    o.body_rate_dps = g_dps + data_.integral_dps;
    o.euler_deg = o.attitude.euler_rad().scaled(kRadToDeg);

    data_signal_(&data_.ahrs);

    // Update our measured rate output.
    data_.bias_count++;
    const auto ticks_per_second = clock_.ticks_per_second();
    if ((now - data_.start_timestamp) > ticks_per_second) {
      data_.start_timestamp += ticks_per_second;
      data_.measured_rate_hz = data_.bias_count;
      data_.bias_count = 0;

      if ((data_.measured_rate_hz < (3 * o.rate_hz / 4)) ||
          (data_.measured_rate_hz > (5 * o.rate_hz / 4))) {
        // If we ever see a bad rate, just latch the error to force a
        // restart.
        o.error = 1;
      }
    }
  }

  Clock& clock_;

  enum State {
    kInitialBias,
    kOperating,
    kNumStates,
  };

  static std::array<std::pair<State, const char*>, kNumStates> StateMapper() {
    return std::array<std::pair<State, const char*>, kNumStates> { {
        { kInitialBias, "kInitialBias" },
        { kOperating, "kOperating" },
      } };
  }

  struct Data {
    State state;
    uint32_t start_timestamp = 0;
    uint32_t bias_count = 0;
    uint32_t measured_rate_hz = 0;
    Point3D integral_dps;
    AhrsData ahrs;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_ENUM(state, StateMapper));
      a->Visit(MJ_NVP(start_timestamp));
      a->Visit(MJ_NVP(bias_count));
      a->Visit(MJ_NVP(measured_rate_hz));
      a->Visit(MJ_NVP(integral_dps));
      a->Visit(MJ_NVP(ahrs));
    }
  };
  Data data_;
  StaticFunction<void ()> data_updater_;
  AhrsDataSignal data_signal_;

  Config config_;
};

MahonyImu::MahonyImu(Pool& pool,
                     Clock& clock,
                     PersistentConfig& config,
                     TelemetryManager& telemetry,
                     ImuDataSignal& imu_signal)
    : impl_(&pool, clock, config, telemetry, imu_signal) {}

MahonyImu::~MahonyImu() {}

const AhrsData& MahonyImu::data() const {
  return impl_->data_.ahrs;
}

AhrsDataSignal* MahonyImu::data_signal() {
  return &impl_->data_signal_;
}
