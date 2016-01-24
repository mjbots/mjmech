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

#include "fire_control.h"

#include "clock.h"
#include "gpio_pin.h"
#include "pwm_pin.h"
#include "telemetry_manager.h"
#include "base/visitor.h"

class FireControl::Impl {
 public:
  Impl(Clock& clock,
       PersistentConfig&,
       TelemetryManager& telemetry,
       GpioPin& laser_enable,
       GpioPin& pwm_enable,
       PwmPin& aeg_pwm,
       PwmPin& agitator_pwm,
       GpioPin& arm_switch,
       GpioPin& arm_led)
      : clock_(clock),
        laser_enable_(laser_enable),
        pwm_enable_(pwm_enable),
        aeg_pwm_(aeg_pwm),
        agitator_pwm_(agitator_pwm),
        arm_switch_(arm_switch),
        arm_led_(arm_led) {
    data_updated_ = telemetry.Register(gsl::ensure_z("fire_control"), &data_);
  }

  void Emit() {
    data_.timestamp = clock_.timestamp();
    data_updated_();
  }

  void Poll100ms() {
    data_.armed = arm_switch_.Read() ? 1 : 0;

    data_.flash_count = (data_.flash_count + 1) % 10;

    if (data_.armed) {
      arm_led_.Set(
          [&]() {
            switch (data_.flash_count) {
              case 0:
              case 2: { return true; }
              default: { return false; }
            }
          }());
    } else {
      arm_led_.Set(false);
    }

    if (data_.fire_time_100ms) {
      data_.fire_time_100ms--;
    } else {
      data_.fire_pwm = 0;
    }

    if (data_.laser_time_100ms) {
      data_.laser_time_100ms--;
    } else {
      data_.agitator_pwm = 0;
      data_.laser_enabled = 0;
    }

    Update();
  }

  void Update() {
    pwm_enable_.Set(data_.fire_pwm || data_.agitator_pwm);

    laser_enable_.Set(data_.laser_enabled != 0);
    aeg_pwm_.Set(data_.fire_pwm * 257);
    agitator_pwm_.Set(data_.agitator_pwm * 257);

    Emit();
  }

  void UnknownCommand(const gsl::cstring_span& command,
                      const CommandManager::Response& response) {
    WriteMessage(gsl::ensure_z("unknown command\r\n"), response);
  }

  void WriteOK(const CommandManager::Response& response) {
    WriteMessage(gsl::ensure_z("OK\r\n"), response);
  }

  void WriteMessage(const gsl::cstring_span& message,
                    const CommandManager::Response& response) {
    AsyncWrite(*response.stream, message, response.callback);
  }

  struct Data {
    uint32_t timestamp = 0;
    int32_t count_100ms = 0;

    uint8_t fire_time_100ms = 0;
    uint8_t fire_pwm = 0;
    uint8_t agitator_pwm = 0;
    uint8_t laser_time_100ms = 0;
    uint8_t laser_enabled = 0;
    uint8_t armed = 0;
    uint8_t flash_count = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(count_100ms));

      a->Visit(MJ_NVP(fire_time_100ms));
      a->Visit(MJ_NVP(fire_pwm));
      a->Visit(MJ_NVP(agitator_pwm));
      a->Visit(MJ_NVP(laser_time_100ms));
      a->Visit(MJ_NVP(laser_enabled));
      a->Visit(MJ_NVP(armed));
      a->Visit(MJ_NVP(flash_count));
    }
  };

  Clock& clock_;
  GpioPin& laser_enable_;
  GpioPin& pwm_enable_;
  PwmPin& aeg_pwm_;
  PwmPin& agitator_pwm_;
  GpioPin& arm_switch_;
  GpioPin& arm_led_;

  Data data_;
  StaticFunction<void ()> data_updated_;
};

FireControl::FireControl(
    Pool& pool, Clock& clock, PersistentConfig& config,
    TelemetryManager& telemetry,
    GpioPin& laser_enable, GpioPin& pwm_enable,
    PwmPin& aeg_pwm, PwmPin& agitator_pwm,
    GpioPin& arm_switch, GpioPin& arm_led)
    : impl_(&pool, clock, config, telemetry,
            laser_enable, pwm_enable, aeg_pwm, agitator_pwm,
            arm_switch, arm_led) {
}

FireControl::~FireControl() {}

void FireControl::SetLaser(bool value) {
  impl_->data_.laser_enabled = value ? 1 : 0;
  impl_->data_.laser_time_100ms = 200;
  impl_->Update();
}

void FireControl::SetFire(uint8_t pwm, uint8_t time_100ms) {
  impl_->data_.fire_time_100ms = time_100ms;
  impl_->data_.fire_pwm = pwm;
  impl_->Update();
}

void FireControl::SetAgitator(uint8_t pwm) {
  impl_->data_.agitator_pwm = pwm;
  impl_->Update();
}

bool FireControl::laser() const {
  return impl_->data_.laser_enabled;
}

uint8_t FireControl::fire_pwm() const {
  return impl_->data_.fire_pwm;
}

uint8_t FireControl::fire_time_100ms() const {
  return impl_->data_.fire_time_100ms;
}

uint8_t FireControl::agitator_pwm() const {
  return impl_->data_.agitator_pwm;
}

void FireControl::Command(const gsl::cstring_span& command,
                          const CommandManager::Response& response) {
  Tokenizer tokenizer(command, " ");
  const auto cmd = tokenizer.next();
  if (cmd == gsl::ensure_z("fire")) {
    const auto pwm_str = tokenizer.next();
    const auto time_100ms_str = tokenizer.next();
    SetFire(std::strtol(pwm_str.data(), nullptr, 0),
            std::strtol(time_100ms_str.data(), nullptr, 0));
    impl_->WriteOK(response);
  } else if (cmd == gsl::ensure_z("laser")) {
    const auto laser_val = tokenizer.next();
    SetLaser(laser_val == gsl::ensure_z("1"));
    impl_->WriteOK(response);
  } else if (cmd == gsl::ensure_z("agitator")) {
    const auto agitator_str = tokenizer.next();
    SetAgitator(std::strtol(agitator_str.data(), nullptr, 0));
    impl_->WriteOK(response);
  } else {
    impl_->UnknownCommand(command, response);
  }
}

void FireControl::PollMillisecond() {
  if (impl_->data_.count_100ms <= 0) {
    impl_->data_.count_100ms = 100;
    impl_->Poll100ms();
  } else {
    impl_->data_.count_100ms--;
  }
}
