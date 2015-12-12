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

uint32_t int_log2(uint32_t value) {
  int result = 0;
  while (value >>= 1) {
    result += 1;
  }
  return result;
}

uint32_t GetPinInputMask(uint16_t pin) {
  int bit = int_log2(pin);
  return (1 << (bit * 2)) | (1 << (bit * 2 + 1));
}

uint32_t GetPinOutputBits(uint16_t pin) {
  // Put the pin back into alternate function mode, or 0b10.
  int bit = int_log2(pin);
  return (1 << (bit * 2 + 1));
}
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

void UART_Stream_IRQHandler(UART_HandleTypeDef* huart) {
  for (const auto& item: g_stream_registry) {
    if (item.huart == huart) {
      item.stream->Interrupt();
      return;
    }
  }
  assert(false);
}

}

UartStream::UartStream(UART_HandleTypeDef* huart,
                       GPIO_TypeDef* tx_gpio,
                       uint16_t tx_pin)
    : huart_(huart),
      gpio_conf_register_(tx_gpio ? &tx_gpio->MODER : nullptr),
      gpio_input_mask_(GetPinInputMask(tx_pin)),
      gpio_output_bits_(GetPinOutputBits(tx_pin)) {

  if (gpio_conf_register_) {
    (*gpio_conf_register_) &= ~gpio_input_mask_;
  }

  [&]() {
    for (auto& item: g_stream_registry) {
      if (item.huart == nullptr) {
        item.huart = huart;
        item.stream = this;
        return;
      }
    }

    assert(false);
  }();

  huart_->Instance->CR1 |= USART_CR1_RXNEIE;
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
  rx_buffer_ = buffer;
}

namespace {
class InterruptGuard {
 public:
  InterruptGuard(UART_HandleTypeDef* uart) : uart_(uart) {
    uart_->Instance->CR1 &= ~USART_CR1_RXNEIE;
  }

  ~InterruptGuard() {
    uart_->Instance->CR1 |= USART_CR1_RXNEIE;
  }

 private:
  UART_HandleTypeDef* const uart_;
};
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

    // Only put ourselves back into input mode if we didn't
    // immediately schedule another transmission.
    if (!tx_callback_.valid()) {
      if (gpio_conf_register_) {
        (*gpio_conf_register_) &= ~gpio_input_mask_;
      }
    }
  }

  if (!buffer_.empty()) {
    if (rx_callback_.valid()) {
      Expects(rx_buffer_.size() != 0);

      std::size_t pos = 0;
      while (!buffer_.empty() && pos < rx_buffer_.size()) {
        rx_buffer_.data()[pos] = buffer_.front();
        buffer_.pop_front();
        pos++;
      }

      auto callback = rx_callback_;
      rx_callback_ = SizeCallback();
      callback(0, pos);
      assert(rx_callback_.valid());
    }
  }
}

void UartStream::AsyncWriteSome(const gsl::cstring_span& buffer,
                                SizeCallback callback) {
  assert(!tx_callback_.valid());

  if (gpio_conf_register_) {
    (*gpio_conf_register_) |= gpio_output_bits_;
  }

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

void UartStream::Interrupt() {
  const uint32_t sr = huart_->Instance->SR;
  if (sr & USART_SR_RXNE ||
      sr & USART_SR_ORE) {
    const uint8_t data = huart_->Instance->DR;
    if (!buffer_.full()) {
      buffer_.push_back(data);
    } else {
      rx_overflow_ = true;
    }
  }
}
