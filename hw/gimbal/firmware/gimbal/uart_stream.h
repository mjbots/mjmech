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

class UartStream : public AsyncStream {
 public:
  UartStream(UART_HandleTypeDef*);
  virtual ~UartStream();

  virtual void AsyncReadSome(const gsl::string_span&, SizeCallback) override;
  virtual void AsyncWriteSome(const gsl::cstring_span&, SizeCallback) override;

  void Poll();

  // The following are used internally only.
  void TransmitComplete();
  void ReceiveComplete();

 private:
  UART_HandleTypeDef* const huart_;
  SizeCallback tx_callback_;
  std::size_t tx_size_ = 0;

  volatile bool rx_complete_ = false;
  volatile bool tx_complete_ = false;

  SizeCallback rx_callback_;
};
