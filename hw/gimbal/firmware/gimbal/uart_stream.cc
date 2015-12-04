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

#include "uart_stream.h"

#include <assert.h>

namespace {
struct Registry {
  UART_HandleTypeDef* huart = nullptr;
  UartStream* stream = nullptr;
};

Registry g_stream_registry[3];
}

extern "C" {

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  for (const auto& item: g_stream_registry) {
    if (item.huart == huart) {
      item.stream->TransmitComplete();
      return;
    }
  }
  assert(false);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  for (const auto& item: g_stream_registry) {
    if (item.huart == huart) {
      item.stream->ReceiveComplete();
      return;
    }
  }
  assert(false);
}

}

UartStream::UartStream(UART_HandleTypeDef* huart) : huart_(huart) {
  for (auto& item: g_stream_registry) {
    if (item.huart == nullptr) {
      item.huart = huart;
      item.stream = this;
      return;
    }
  }

  assert(false);
}

UartStream::~UartStream() {
  for (auto& item: g_stream_registry) {
    if (item.huart == huart_) {
      item.huart = nullptr;
      item.stream = nullptr;
      return;
    }
  }
  assert(false);
}

void UartStream::AsyncReadSome(const gsl::string_span& buffer,
                               SizeCallback callback) {
  assert(!rx_callback_.valid());

  // We only ever ask for 1 byte at a time from the serial port.
  rx_callback_ = callback;
  HAL_UART_Receive_DMA(
      huart_,
      reinterpret_cast<uint8_t*>(buffer.data()),
      1);
}

void UartStream::Poll() {
  if (tx_complete_) {
    tx_complete_ = false;

    if (tx_callback_.valid()) {

      auto callback = tx_callback_;
      tx_callback_ = SizeCallback();
      auto size = tx_size_;
      tx_size_ = 0;
      callback(0, size);
    }
  }

  if (rx_complete_) {
    rx_complete_ = false;

    if (rx_callback_.valid()) {
      auto callback = rx_callback_;
      rx_callback_ = SizeCallback();
      callback(0, 1);
    }
  }
}

void UartStream::ReceiveComplete() {
  rx_complete_ = true;
}

void UartStream::AsyncWriteSome(const gsl::cstring_span& buffer,
                                SizeCallback callback) {
  assert(!tx_callback_.valid());

  tx_callback_ = callback;
  tx_size_ = buffer.size();
  HAL_UART_Transmit_DMA(
      huart_,
      const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(buffer.data())),
      buffer.size());
}

void UartStream::TransmitComplete() {
  tx_complete_ = true;
}
