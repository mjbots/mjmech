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

#include "usart.h"

#include "async_stream.h"
#include "circular_buffer.h"

class UartStream : public AsyncStream {
 public:
  /// @p tx_gpio may be nullptr, in which case the output will not be
  /// tristated when the transmitter is idle.
  UartStream(UART_HandleTypeDef*, GPIO_TypeDef* tx_gpio, uint16_t tx_pin);
  virtual ~UartStream();

  virtual void AsyncReadSome(const gsl::string_span&, SizeCallback) override;
  virtual void AsyncWriteSome(const gsl::cstring_span&, SizeCallback) override;

  void Poll();

  // The following are used internally only.
  void TransmitComplete();
  void Interrupt();

 private:
  UART_HandleTypeDef* const huart_;
  SizeCallback tx_callback_;
  std::size_t tx_size_ = 0;
  volatile uint32_t* const gpio_conf_register_;
  const uint32_t gpio_input_mask_;
  const uint32_t gpio_output_bits_;

  circular_buffer<char, 32> buffer_;

  volatile bool tx_complete_ = false;
  volatile bool rx_overflow_ = false;

  SizeCallback rx_callback_;
  gsl::string_span rx_buffer_;
};
