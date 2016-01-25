// Copyright 2015-2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "gimbal_stabilizer.h"

#include "bldc_pwm.h"
#include "clock.h"
#include "gpio_pin.h"
#include "persistent_config.h"
#include "telemetry_manager.h"

namespace {
template <typename T>
T Limit(T value, T max) {
  T result = value;
  if (result > max) { result = max; }
  if (result < -max) { result = -max; }
  return result;
}

template <typename T>
T Limit(T value, T min, T max) {
  T result = value;
  if (result > max) { result = max; }
  if (result < min) { result = min; }
  return result;
}

struct ChannelConfig {
  int8_t motor = -1;
  float power = 0.1f;
  PID::Config pid;
  float max_slew_dps = 60.0f;

  /// 0 is traditional open loop
  /// 1 uses bldc encoder to apply fixed torque
  /// 2 just applies a fixed motor phase based on target
  uint8_t mode = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(motor));
    a->Visit(MJ_NVP(power));
    a->Visit(MJ_NVP(pid));
    a->Visit(MJ_NVP(max_slew_dps));
    a->Visit(MJ_NVP(mode));
  }

  ChannelConfig() {
    pid.kp = 0.2f;
    pid.ki = 0.1f;
    pid.kd = 0.01f;
    pid.iratelimit = 1.0f;
    pid.ilimit = 1000.0f;
    pid.kpkd_limit = 0.25f;
    pid.sign = -1;
  }
};

struct LimitConfig {
  float min_deg = 0.0f;
  float max_deg = 0.0f;

  bool empty() const {
    return min_deg == 0.0f && max_deg == 0.0f;
  }

  void Limit(float* value) const {
    if (empty()) { return; }
    *value = ::Limit(*value, min_deg, max_deg);
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(min_deg));
    a->Visit(MJ_NVP(max_deg));
  }
};

struct Config {
  float initialization_period_s = 1.0f;
  float watchdog_period_s = 0.1f;
  bool boost = false;
  ChannelConfig pitch;
  ChannelConfig yaw;

  LimitConfig pitch_limit;
  LimitConfig abs_yaw_limit;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(initialization_period_s));
    a->Visit(MJ_NVP(watchdog_period_s));
    a->Visit(MJ_NVP(boost));
    a->Visit(MJ_NVP(pitch));
    a->Visit(MJ_NVP(yaw));
    a->Visit(MJ_NVP(pitch_limit));
    a->Visit(MJ_NVP(abs_yaw_limit));
  }

  Config() {
    pitch.motor = 2;
    yaw.motor = 1;
    pitch_limit.min_deg = -15.0f;
    pitch_limit.max_deg = 30.0f;
  }
};

}

class GimbalStabilizer::Impl {
 public:
  Impl(Clock& clock,
       PersistentConfig& config,
       TelemetryManager& telemetry,
       AhrsDataSignal& ahrs_signal,
       GpioPin& boost_enable,
       GpioPin& motor_enable,
       BldcPwm& motor1,
       BldcPwm& motor2,
       BldcEncoderDataSignal& yaw_encoder_signal,
       GpioPin& torque_led)
      : clock_(clock),
        boost_enable_(boost_enable),
        motor_enable_(motor_enable),
        motor1_(motor1),
        motor2_(motor2),
        torque_led_(torque_led) {
    config.Register(gsl::ensure_z("gimbal"), &config_, [](){});
    data_updater_ = telemetry.Register(gsl::ensure_z("gimbal"), &data_);
    ahrs_signal.Connect(
        [this](const AhrsData* data) { this->HandleAhrs(data); });
    yaw_encoder_signal.Connect(
        [this](const BldcEncoderData* data) { this->HandleYawEncoder(data); });
    torque_led_.Set(false);
  }

  void HandleAhrs(const AhrsData* data) {
    ahrs_data_ = *data;

    switch (data_.state) {
      case kInitializing: { DoInitializing(data); break; }
      case kOperating: { DoOperating(data); break; }
      case kFault: { DoFault(); break; }
      case kNumStates: { assert(false); break; }
    }

    data_updater_();
  }

  void HandleYawEncoder(const BldcEncoderData* data) {
    yaw_encoder_ = *data;
  }

  void DoInitializing(const AhrsData* data) {
    boost_enable_.Set(false);
    motor_enable_.Set(false);

    if (data->error) {
      data_.start_timestamp = 0;
      return;
    }

    if (data_.start_timestamp == 0) {
      data_.start_timestamp = clock_.timestamp();
      return;
    }

    const uint32_t now = clock_.timestamp();
    const float elapsed_s =
        static_cast<float>(now - data_.start_timestamp) /
        clock_.ticks_per_second();
    if (elapsed_s > config_.initialization_period_s) {
      data_.desired_deg.pitch = 0.0f;
      data_.desired_deg.yaw = data->euler_deg.yaw;
      data_.target_deg.pitch = data->euler_deg.pitch;
      data_.target_deg.yaw = data->euler_deg.yaw;
      data_.last_ahrs_update = now;
      data_.state = kOperating;
      return;
    }
  }

  void UpdateSlew(float* target_deg, float desired_deg,
                  uint16_t rate_hz, float max_slew_dps) {
    const float max_delta_deg = max_slew_dps / rate_hz;
    const float delta_deg = Limit(desired_deg - *target_deg, max_delta_deg);
    *target_deg += delta_deg;
  }

  void DoOperating(const AhrsData* data) {
    if (data->error) {
      data_.last_fault_reason = 1;
      DoFault();
      return;
    }

    data_.last_ahrs_update = data->timestamp;
    boost_enable_.Set(config_.boost);
    motor_enable_.Set(data_.torque_on);
    torque_led_.Set(data_.torque_on);

    data_.desired_deg.pitch += data_.desired_body_rate_dps.x / data->rate_hz;
    config_.pitch_limit.Limit(&data_.desired_deg.pitch);

    // Body rate Z and yaw have opposite signs.
    data_.desired_deg.yaw -= data_.desired_body_rate_dps.z / data->rate_hz;
    if (CheckYawEncoderRecent() && !config_.abs_yaw_limit.empty()) {
      const float yaw_min_deg = data->euler_deg.yaw -
          (yaw_encoder_.position_deg - config_.abs_yaw_limit.min_deg);
      const float yaw_max_deg = data->euler_deg.yaw +
          (config_.abs_yaw_limit.max_deg - yaw_encoder_.position_deg);
      if (data_.desired_deg.yaw < yaw_min_deg) {
        data_.desired_deg.yaw = yaw_min_deg;
      } else if (data_.desired_deg.yaw > yaw_max_deg) {
        data_.desired_deg.yaw = yaw_max_deg;
      }
    }

    UpdateSlew(&data_.target_deg.pitch, data_.desired_deg.pitch,
               data->rate_hz, config_.pitch.max_slew_dps);
    UpdateSlew(&data_.target_deg.yaw, data_.desired_deg.yaw,
               data->rate_hz, config_.yaw.max_slew_dps);

    const float pitch_control_command =
        pitch_pid_.Apply(data->euler_deg.pitch, data_.target_deg.pitch,
                         data->body_rate_dps.x, data_.desired_body_rate_dps.x,
                         data->rate_hz);
    // For yaw, the sign of body rate in Z is opposite that of yaw.
    // To make the PID controller have the same signs for everything,
    // we invert it before passing it in.
    const float yaw_control_command =
        yaw_pid_.Apply(data->euler_deg.yaw, data_.target_deg.yaw,
                       -data->body_rate_dps.z, -data_.desired_body_rate_dps.z,
                       data->rate_hz);

    // For the open loop case, our integral should be wrapped to be
    // between 0 and 1.0.
    const auto wrap_integral = [](float* integral) {
      *integral = std::fmod(*integral, 1.0f);
      if (*integral < 0.0f) { (*integral) += 1.0f; }
    };
    if (config_.pitch.mode == 0) {
      wrap_integral(&data_.pitch.integral);
    }
    if (config_.yaw.mode == 0) {
      wrap_integral(&data_.yaw.integral);
    }

    const float pitch_command = [&, mode=config_.pitch.mode]() {
      if (mode == 0) {
        return pitch_control_command;
      } else if (mode == 1) {
        // TODO jpieper: When we support bldc encoders on pitch, add it
        // here.
        return 0.0f;
      } else if (mode == 2) {
        return data_.target_deg.pitch / 360.0f;
      }
      return 0.0f;
    }();

    const float yaw_command = [&, mode=config_.yaw.mode]() {
      if (mode == 0) {
        return yaw_control_command;
      } else if (mode == 1) {
        return yaw_control_command + yaw_encoder_.phase;
      } else if (mode == 2) {
        return data_.target_deg.yaw / 360.0f;
      }

      return 0.0f;
    }();

    const auto pwm = [&](float command, float power_in, float phase) {
      const float power = Limit(power_in, 0.0f, 1.0f);
      const float fresult = (
          (std::sin((command + phase) *
                    2 * mjmech::base::kPi) + 1.0f) *
          power * 32767);
      return static_cast<uint16_t>(
          std::max(static_cast<uint32_t>(0),
                   std::min(static_cast<uint32_t>(65535),
                            static_cast<uint32_t>(fresult))));
    };
    const float ppower = config_.pitch.power;
    const float pitch1 = pwm(pitch_command, ppower, 0.0f);
    const float pitch2 = pwm(pitch_command, ppower, 1.0f / 3.0f);
    const float pitch3 = pwm(pitch_command, ppower, 2.0f / 3.0f);

    const float ypower = config_.yaw.power;
    const float yaw1 = pwm(yaw_command, ypower, 0.0f);
    const float yaw2 = pwm(yaw_command, ypower, 1.0f / 3.0f);
    const float yaw3 = pwm(yaw_command, ypower, 2.0f / 3.0f);

    pitch_motor().Set(pitch1, pitch2, pitch3);
    yaw_motor().Set(yaw1, yaw2, yaw3);

    data_.pitch_command = pitch_command;
    data_.yaw_command = yaw_command;
  }

  void DoFault() {
    data_.state = kFault;
    data_.torque_on = false;
    boost_enable_.Set(false);
    motor_enable_.Set(false);
    data_.desired_body_rate_dps = Point3D();
    motor1_.Set(0, 0, 0);
    motor2_.Set(0, 0, 0);
    torque_led_.Set(false);
  }

  void PollMillisecond() {
    // If last_ahrs_update is too stale, fault.
    switch (data_.state) {
      case kOperating: {
        const float elapsed_s = static_cast<float>(
            clock_.timestamp() -
            data_.last_ahrs_update) / clock_.ticks_per_second();
        if (elapsed_s > config_.watchdog_period_s && data_.torque_on) {
          data_.last_fault_reason = 2;
          DoFault();
        }
        break;
      }
      case kInitializing:
      case kFault: {
        break;
      }
      case kNumStates: { assert(false); break; }
    }
  }

  BldcPwm& pitch_motor() {
    if (config_.pitch.motor == 1) { return motor1_; }
    else if (config_.pitch.motor == 2) { return motor2_; }
    return motor1_;
  }

  BldcPwm& yaw_motor() {
    if (config_.yaw.motor == 1) { return motor1_; }
    else if (config_.yaw.motor == 2) { return motor2_; }
    return motor2_;
  }

  void SetImuAttitude(float pitch_deg, float yaw_deg) {
    // TODO jpieper: Make these slew.
    data_.desired_deg.pitch = pitch_deg;
    data_.desired_deg.yaw = yaw_deg;
  }

  void SetImuRate(float pitch_dps, float yaw_dps) {
    data_.desired_body_rate_dps.x =
        Limit(pitch_dps, config_.pitch.max_slew_dps);
    data_.desired_body_rate_dps.z =
        Limit(yaw_dps, config_.yaw.max_slew_dps);
  }

  void AttitudeCommand(const gsl::cstring_span& command,
                       const CommandManager::Response& response) {
    Tokenizer tokenizer(command, " ");
    auto pitch_str = tokenizer.next();
    if (pitch_str.size() == 0) {
      BadFormat(response);
      return;
    }

    auto yaw_str = tokenizer.next();
    if (yaw_str.size() == 0) {
      BadFormat(response);
      return;
    }

    // Unfortunately, these APIs don't give a trivial way to check for
    // conversion errors when dealing with non-null terminated
    // strings.  Thus, ignore errors.
    const float pitch_deg = std::strtod(pitch_str.data(), nullptr);
    const float yaw_deg = std::strtod(yaw_str.data(), nullptr);

    SetImuAttitude(pitch_deg, yaw_deg);

    WriteOK(response);
  }

  void RateCommand(const gsl::cstring_span& command,
                   const CommandManager::Response& response) {
    Tokenizer tokenizer(command, " ");
    auto pitch_rate_str = tokenizer.next();
    if (pitch_rate_str.size() == 0) {
      BadFormat(response);
      return;
    }

    auto yaw_rate_str = tokenizer.next();
    if (yaw_rate_str.size() == 0) {
      BadFormat(response);
      return;
    }

    const float pitch_dps = std::strtod(pitch_rate_str.data(), nullptr);
    const float yaw_dps = std::strtod(yaw_rate_str.data(), nullptr);

    SetImuRate(pitch_dps, yaw_dps);

    WriteOK(response);
  }

  void WriteOK(const CommandManager::Response response) {
    WriteMessage(gsl::ensure_z("OK\r\n"), response);
  }

  void BadFormat(const CommandManager::Response& response) {
    WriteMessage(gsl::ensure_z("bad format\r\n"), response);
  }

  void UnknownCommand(const CommandManager::Response& response) {
    WriteMessage(gsl::ensure_z("unknown command\r\n"), response);
  }

  void WriteMessage(const gsl::cstring_span& message,
                    const CommandManager::Response& response) {
    AsyncWrite(*response.stream, message, response.callback);
  }

  bool CheckYawEncoderRecent() const {
    const auto delta = clock_.timestamp() - yaw_encoder_.timestamp;
    const auto max_age = clock_.ticks_per_second() / 100;
    return delta < max_age;
  }

  Clock& clock_;
  GpioPin& boost_enable_;
  GpioPin& motor_enable_;
  BldcPwm& motor1_;
  BldcPwm& motor2_;
  GpioPin& torque_led_;

  Config config_;

  Data data_;
  StaticFunction<void ()> data_updater_;
  PID pitch_pid_{&config_.pitch.pid, &data_.pitch};
  PID yaw_pid_{&config_.yaw.pid, &data_.yaw};
  AhrsData ahrs_data_;
  BldcEncoderData yaw_encoder_;
};

GimbalStabilizer::GimbalStabilizer(
    Pool& pool,
    Clock& clock,
    PersistentConfig& config,
    TelemetryManager& telemetry,
    AhrsDataSignal& ahrs_signal,
    GpioPin& boost_enable,
    GpioPin& motor_enable,
    BldcPwm& motor1,
    BldcPwm& motor2,
    BldcEncoderDataSignal& yaw_encoder_signal,
    GpioPin& torque_led)
    : impl_(&pool, clock, config, telemetry, ahrs_signal,
            boost_enable, motor_enable,
            motor1, motor2, yaw_encoder_signal, torque_led) {}

GimbalStabilizer::~GimbalStabilizer() {}

void GimbalStabilizer::PollMillisecond() {
  impl_->PollMillisecond();
}

void GimbalStabilizer::Reset() {
  impl_->data_.state = kInitializing;
  impl_->data_.start_timestamp = 0;
  impl_->data_.pitch = PID::State();
  impl_->data_.yaw = PID::State();
  impl_->data_.torque_on = false;
}

void GimbalStabilizer::SetTorque(bool v) {
  const bool old_torque = impl_->data_.torque_on;
  impl_->data_.torque_on = v;
  if (!old_torque && v) {
    // Always start with the target yaw and pitch where they are now,
    // so we make a smooth motion when turning on.
    impl_->data_.target_deg.pitch = impl_->ahrs_data_.euler_deg.pitch;
    impl_->data_.target_deg.yaw = impl_->ahrs_data_.euler_deg.yaw;
  }

  if (!v && impl_->data_.state == kFault) {
    impl_->data_.state = kInitializing;
    impl_->data_.start_timestamp = impl_->clock_.timestamp();
  }
}

const GimbalStabilizer::Data& GimbalStabilizer::data() const {
  return impl_->data_;
}

void GimbalStabilizer::SetImuAttitude(float pitch_deg, float yaw_deg) {
  impl_->SetImuAttitude(pitch_deg, yaw_deg);
}

void GimbalStabilizer::Command(const gsl::cstring_span& command,
                               const CommandManager::Response& response) {
  Tokenizer tokenizer(command, " ");
  auto cmd = tokenizer.next();
  if (cmd == gsl::ensure_z("on")) {
    SetTorque(true);
    impl_->WriteOK(response);
  } else if (cmd == gsl::ensure_z("off")) {
    SetTorque(false);
    impl_->WriteOK(response);
  } else if (cmd == gsl::ensure_z("att")) {
    impl_->AttitudeCommand(tokenizer.remaining(), response);
  } else if (cmd == gsl::ensure_z("rate")) {
    impl_->RateCommand(tokenizer.remaining(), response);
  } else {
    impl_->UnknownCommand(response);
  }
}
