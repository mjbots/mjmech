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

#include "gpio.h"
#include "iwdg.h"

extern "C" {
void cpp_gimbal_main() {
  // For now, lets try to flash an LED with no sleeps or anything.
  uint32_t old_tick = 0;
  int cycle = 0;
  while (1) {
    HAL_IWDG_Refresh(&hiwdg);
    uint32_t new_tick = HAL_GetTick();
    if (new_tick != old_tick && (new_tick % 250) == 0) {
      cycle = (cycle + 1) % 8;
      if (cycle & 1) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
      }
      if (cycle & 2) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
      }
      if (cycle & 4) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
      }
    }
    old_tick = new_tick;
  }
}
}
