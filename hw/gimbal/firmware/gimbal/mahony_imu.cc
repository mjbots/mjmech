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
    data_signal_(&data_.ahrs);
  }

  void DoInitialBias(const ImuData* data) {
    data_.state = kOperating;
  }

  void DoOperating(const ImuData* data) {
    const Point3D a_g = data->accel_g;
    const Point3D a = a_g.scaled(1.0 / a_g.length());

    const float kDegToRad = mjmech::base::kPi / 180.0;
    // TODO jpieper: Apply initial bias estimate.
    const Point3D g = data->gyro_dps.scaled(kDegToRad);

    // Start initializing our output structure.
    auto& o = data_.ahrs;
    o.rate_hz = data->rate_hz;

    // Estimate the direction of gravity.
    Point3D rotated = o.attitude.conjugated().
                      Rotate(Point3D(0.0, 0.0, 0.5));

    // Then, the error between that estimate, and what we actually
    // saw.
    const Point3D err = a.cross(rotated);

    // Determine integrative terms.
    data_.integral_rps += err.scaled(config_.ki / data->rate_hz);

    // Apply the corrections.
    const Point3D cg = g + err.scaled(config_.kp) + data_.integral_rps;

    // Now, multiply our existing quaternion with the integrated rate
    // of change.
    o.attitude =
        o.attitude *
        Quaternion::IntegrateRotationRate(cg, 1.0f / data->rate_hz);

    o.body_rate_dps = (g + data_.integral_rps).scaled(1.0f / kDegToRad);
    o.euler_deg = o.attitude.euler_rad().scaled(1.0f / kDegToRad);
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
    Point3D integral_rps;
    float integral_x = 0.0f;
    float integral_y = 0.0f;
    float integral_z = 0.0f;
    AhrsData ahrs;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_ENUM(state, StateMapper));
      a->Visit(MJ_NVP(integral_rps));
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
