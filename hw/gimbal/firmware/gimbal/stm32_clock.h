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

#include "tim.h"

#include "clock.h"

class Stm32Clock : public Clock {
 public:
  Stm32Clock() {
    HAL_TIM_Base_Start(&htim5);
  }

  virtual ~Stm32Clock() {}

  virtual uint32_t timestamp() const {
    return __HAL_TIM_GET_COUNTER(&htim5);
  }
};
