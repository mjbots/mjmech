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

#include "stm32_bldc_pwm.h"

#include <assert.h>

namespace {
volatile uint32_t* FindCCR(TIM_HandleTypeDef* tim,
                  uint16_t phase_a_channel) {
  if (phase_a_channel == TIM_CHANNEL_1) { return &tim->Instance->CCR1; }
  else if (phase_a_channel == TIM_CHANNEL_2) { return &tim->Instance->CCR2; }
  else if (phase_a_channel == TIM_CHANNEL_3) { return &tim->Instance->CCR3; }
  else if (phase_a_channel == TIM_CHANNEL_4) { return &tim->Instance->CCR4; }
  assert(false);
}

void StartTimer(TIM_HandleTypeDef* tim,
                uint16_t pin) {
  assert(tim->Instance != TIM1); // TIM1 needs special handling
  HAL_TIM_PWM_Start(tim, pin);
}
}

Stm32BldcPwm::Stm32BldcPwm(TIM_HandleTypeDef* phase_a_tim,
                           uint16_t phase_a_channel,
                           TIM_HandleTypeDef* phase_b_tim,
                           uint16_t phase_b_channel,
                           TIM_HandleTypeDef* phase_c_tim,
                           uint16_t phase_c_channel)
: phase_a_ccr_(FindCCR(phase_a_tim, phase_a_channel)),
  phase_b_ccr_(FindCCR(phase_b_tim, phase_b_channel)),
  phase_c_ccr_(FindCCR(phase_c_tim, phase_c_channel)) {

  StartTimer(phase_a_tim, phase_a_channel);
  StartTimer(phase_b_tim, phase_b_channel);
  StartTimer(phase_c_tim, phase_c_channel);
}

Stm32BldcPwm::~Stm32BldcPwm() {}

