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

#include "gpio_pin.h"

#include "gpio.h"

class Stm32GpioPin : public GpioPin {
 public:
  Stm32GpioPin(GPIO_TypeDef* gpio, uint16_t pin, bool invert=false)
      : gpio_(gpio), pin_(pin), invert_(invert) {}
  virtual ~Stm32GpioPin() {}

  void Set(bool value) override {
    HAL_GPIO_WritePin(
        gpio_, pin_, (value ^ invert_) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }

  bool Read() const override {
    return HAL_GPIO_ReadPin(gpio_, pin_) ? true : false;
  }

 private:
  GPIO_TypeDef* const gpio_;
  const uint16_t pin_;
  const bool invert_;
};
