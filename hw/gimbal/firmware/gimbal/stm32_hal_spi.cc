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

#include "stm32_hal_spi.h"

#include <assert.h>

#include "spi.h"

namespace {
SPI_HandleTypeDef* SelectSPI(int n) {
  if (n == 1) { return &hspi1; }
  assert(false);
}

struct Registry {
  SPI_HandleTypeDef* spi = nullptr;
  Stm32HalSPI::Impl* impl = nullptr;
};

std::array<Registry, 4> g_registry;
}

class Stm32HalSPI::Impl {
 public:
  Impl(int spi_number, GPIO_TypeDef* cs_gpio, uint16_t cs_pin)
      : spi_(SelectSPI(spi_number)),
        cs_gpio_(cs_gpio),
        cs_pin_(cs_pin) {
    for (auto& element: g_registry) {
      if (element.spi == nullptr) {
        element.spi = spi_;
        element.impl = this;
        return;
      }
    }
    assert(false);
  }

  ~Impl() {
    for (auto& element: g_registry) {
      if (element.impl == this) {
        element.spi = nullptr;
        element.impl = nullptr;
        return;
      }
    }
    assert(false);
  }

  void Poll() {
    if (complete_flag_ == kIdle) { return; }

    // Reset the chip select pin.
    HAL_GPIO_WritePin(cs_gpio_, cs_pin_, GPIO_PIN_SET);

    const int code = [&]() {
      switch (complete_flag_) {
        case kCompleted: { return 0; }
        case kError: { return 1; }
        case kIdle: { assert(false); }
      }
      assert(false);
      return 0;
    }();

    const auto callback = callback_;
    callback_ = ErrorCallback();
    complete_flag_ = kIdle;
    callback(code);
  }

  void Complete() { complete_flag_ = kCompleted; }
  void Error() { error_code_ = 1; complete_flag_ = kError; }

  SPI_HandleTypeDef* const spi_;
  GPIO_TypeDef* const cs_gpio_;
  const uint16_t cs_pin_;

  ErrorCallback callback_;

  enum CompleteFlag {
    kIdle,
    kCompleted,
    kError,
  };

  volatile CompleteFlag complete_flag_ = kIdle;
  volatile int error_code_ = 0;
};

Stm32HalSPI::Stm32HalSPI(Pool& pool, int spi_number,
                         GPIO_TypeDef* cs_gpio, uint16_t cs_pin)
: impl_(&pool, spi_number, cs_gpio, cs_pin) {}

Stm32HalSPI::~Stm32HalSPI() {}

void Stm32HalSPI::AsyncTransaction(const gsl::cstring_span& tx_buffer,
                                   const gsl::string_span& rx_buffer,
                                   ErrorCallback callback) {
  assert(!impl_->callback_.valid());
  assert(tx_buffer.size() == rx_buffer.size());
  assert(impl_->complete_flag_ == Impl::kIdle);

  impl_->callback_ = callback;

  // Do chip select.
  HAL_GPIO_WritePin(impl_->cs_gpio_, impl_->cs_pin_, GPIO_PIN_RESET);

  auto result = HAL_SPI_TransmitReceive_DMA(
      impl_->spi_,
      reinterpret_cast<uint8_t*>(const_cast<char*>(tx_buffer.data())),
      reinterpret_cast<uint8_t*>(rx_buffer.data()),
      tx_buffer.size());
  if (result != 0) {
    impl_->complete_flag_ = Impl::kError;
    impl_->error_code_ = result;
  }
}

void Stm32HalSPI::Poll() {
  impl_->Poll();
}

extern "C" {
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
  for (auto& element: g_registry) {
    if (element.spi == hspi) { element.impl->Complete(); }
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
  for (auto& element: g_registry) {
    if (element.spi == hspi) { element.impl->Error(); }
  }
}
}
