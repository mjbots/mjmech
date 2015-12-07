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

#include "async_spi.h"

#include "gpio.h"

#include "pool_ptr.h"

class Stm32HalSPI : public AsyncSPI {
 public:
  Stm32HalSPI(Pool&, int spi_number,
              GPIO_TypeDef* cs_gpio, uint16_t cs_pin);
  virtual ~Stm32HalSPI();

  void AsyncTransaction(const gsl::cstring_span& tx_buffer,
                        const gsl::string_span& rx_buffer,
                        ErrorCallback) override;

  void Poll();

  class Impl;
 private:
  PoolPtr<Impl> impl_;
};
