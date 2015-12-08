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

#include "bldc_pwm.h"

#include "tim.h"

class Stm32BldcPwm : public BldcPwm {
 public:
  Stm32BldcPwm(TIM_HandleTypeDef* phase_a_tim,
               uint16_t phase_a_channel,
               TIM_HandleTypeDef* phase_b_tim,
               uint16_t phase_b_channel,
               TIM_HandleTypeDef* phase_c_tim,
               uint16_t phase_c_channel);

  ~Stm32BldcPwm();

  void Set(uint16_t phase_a,
           uint16_t phase_b,
           uint16_t phase_c) override {
    *phase_a_ccr_ = phase_a >> 5;
    *phase_b_ccr_ = phase_b >> 5;
    *phase_c_ccr_ = phase_c >> 5;
  }

 private:
  volatile uint32_t* const phase_a_ccr_;
  volatile uint32_t* const phase_b_ccr_;
  volatile uint32_t* const phase_c_ccr_;
};
