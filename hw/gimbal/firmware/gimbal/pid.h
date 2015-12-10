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

#include "base/visitor.h"

class PID {
 public:
  struct Config {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float ilimit = 0.0f;
    float kpkd_limit = -1;
    int8_t sign = 1;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(kp));
      a->Visit(MJ_NVP(ki));
      a->Visit(MJ_NVP(kd));
      a->Visit(MJ_NVP(ilimit));
      a->Visit(MJ_NVP(kpkd_limit));
      a->Visit(MJ_NVP(sign));
    }
  };

  struct State {
    float integral = 0.0f;

    // The following are not actually part of the "state", but are
    // present for purposes of being logged with it.
    float error = 0.0f;
    float error_rate = 0.0f;
    float command = 0.0f;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(integral));
      a->Visit(MJ_NVP(error));
      a->Visit(MJ_NVP(error_rate));
      a->Visit(MJ_NVP(command));
    }
  };

  PID(const Config* config, State* state)
      : config_(config), state_(state) {}

  float Apply(float measured, float desired,
              float measured_rate, float desired_rate,
              int rate_hz) {
    state_->error = measured - desired;
    state_->error_rate = measured_rate - desired_rate;

    state_->integral += state_->error * config_->ki / rate_hz;
    if (state_->integral > config_->ilimit) {
      state_->integral = config_->ilimit;
    } else if (state_->integral < -config_->ilimit) {
      state_->integral = -config_->ilimit;
    }

    float kpkd =
        config_->kp * state_->error +
        config_->kd * state_->error_rate;

    if (config_->kpkd_limit >= 0.0) {
      if (kpkd > config_->kpkd_limit) {
        kpkd = config_->kpkd_limit;
      } else if (kpkd < -config_->kpkd_limit) {
        kpkd = -config_->kpkd_limit;
      }
    }

    state_->command = config_->sign * (kpkd + state_->integral);

    return state_->command;
  }

 private:
  const Config* const config_;
  State* const state_;
};
