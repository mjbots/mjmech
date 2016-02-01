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
      : gpio_(gpio),
        pin_(pin),
        invert_(invert),
        shift_(GetShift(pin)),
        mask_(0x3 << shift_) {}
  virtual ~Stm32GpioPin() {}

  void Set(bool value) override {
    HAL_GPIO_WritePin(
        gpio_, pin_, (value ^ invert_) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }

  bool Read() const override {
    return (HAL_GPIO_ReadPin(gpio_, pin_) ? true : false) ^ invert_;
  }

  void SetMode(PinMode mode) override {
    gpio_->MODER = ((gpio_->MODER) & ~mask_) |
        (static_cast<uint32_t>(mode) << shift_);
  }

  static int GetShift(int pin) {
    switch (pin) {
      case GPIO_PIN_0: { return 0; }
      case GPIO_PIN_1: { return 2; }
      case GPIO_PIN_2: { return 4; }
      case GPIO_PIN_3: { return 6; }
      case GPIO_PIN_4: { return 8; }
      case GPIO_PIN_5: { return 10; }
      case GPIO_PIN_6: { return 12; }
      case GPIO_PIN_7: { return 14; }
      case GPIO_PIN_8: { return 16; }
      case GPIO_PIN_9: { return 18; }
      case GPIO_PIN_10: { return 20; }
      case GPIO_PIN_11: { return 22; }
      case GPIO_PIN_12: { return 24; }
      case GPIO_PIN_13: { return 26; }
      case GPIO_PIN_14: { return 28; }
      case GPIO_PIN_15: { return 30; }
    }
    return 0;
  }

 private:
  GPIO_TypeDef* const gpio_;
  const uint16_t pin_;
  const bool invert_;
  const int shift_;
  const uint32_t mask_;
};
