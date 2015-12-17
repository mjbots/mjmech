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

#include <assert.h>

#include "pwm_pin.h"

class Stm32TimexComplementPwm : public PwmPin {
 public:
  Stm32TimexComplementPwm(TIM_HandleTypeDef* htim, uint16_t channel)
      : ccr_(FindCCR(htim, channel)) {
    HAL_TIMEx_PWMN_Start(htim, channel);
  }

  virtual ~Stm32TimexComplementPwm() {}

  virtual void Set(uint16_t value) {
    *ccr_ = value >> 5;
  }

  static volatile uint32_t* FindCCR(
      TIM_HandleTypeDef* htim, uint16_t channel) {
    switch (channel) {
      case TIM_CHANNEL_1: { return &htim->Instance->CCR1; }
      case TIM_CHANNEL_2: { return &htim->Instance->CCR2; }
      case TIM_CHANNEL_3: { return &htim->Instance->CCR3; }
      case TIM_CHANNEL_4: { return &htim->Instance->CCR4; }
    }
    assert(false);
    return nullptr;
  }

  volatile uint32_t* const ccr_;
};
