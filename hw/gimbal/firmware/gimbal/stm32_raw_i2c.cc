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

#include "stm32_raw_i2c.h"

#include <assert.h>

#include "stm32f4xx.h"

// TODO jpieper: Use DMA for the bulk transfers to reduce overhead.

namespace {
typedef Stm32RawI2C::Parameters Parameters;

const uint16_t kSr1Errors = (
            I2C_SR1_TIMEOUT |
            I2C_SR1_PECERR |
            I2C_SR1_OVR |
            I2C_SR1_AF |
            I2C_SR1_ARLO |
            I2C_SR1_BERR);

uint16_t CalculateCCR(uint32_t pclk,
                      uint32_t speed,
                      Parameters::DutyCycle duty_cycle) {
  uint16_t result = 0;
  if (speed <= 100000) {
    // Standard mode.
    result |= 0 << 15;
    result |= std::max(4ul, ((pclk / speed) << 1) & ((1 << 12) - 1));
  } else {
    // Fast mode.
    result |= 1 << 15;
    switch (duty_cycle) {
      case Parameters::kDuty2: {
        result |= std::max(1ul, (pclk / (speed * 3)) & ((1 << 12) - 1));
        break;
      }
      case Parameters::kDuty16_9: {
        result |= 1 << 14;
        result |= std::max(1ul, (pclk / (speed * 25)));
        break;
      }
    }
  }
  return result;
};

I2C_TypeDef* GetI2C(int i2c_number) {
  switch (i2c_number) {
    case 1: { return I2C1; }
    case 2: { return I2C2; }
    case 3: { return I2C3; }
  }
  assert(false);
  return nullptr;
}
}

class Stm32RawI2C::Impl {
 public:
  Impl(int i2c_number, const Parameters& parameters)
      : i2c_(GetI2C(i2c_number)) {
    assert(i2c_number >= 1 && i2c_number <= 3);

    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t freqrange = I2C_FREQRANGE(pclk1);

    // Start out with the peripheral disabled.
    i2c_->CR1 = 0;

    i2c_->CR2 = freqrange;

    i2c_->TRISE = (parameters.speed <= 100000) ?
        (freqrange + 1) :
        (freqrange * 300 / 1000 + 1);

    i2c_->CCR = CalculateCCR(pclk1, parameters.speed, parameters.duty_cycle);

    // Start out enabled, with no fancy options.
    i2c_->CR1 = I2C_CR1_PE;

    // This class does master only, so we don't care about our own
    // addresses.
    i2c_->OAR1 = 0;
    i2c_->OAR2 = 0;

    if (i2c_->SR2 & I2C_SR2_BUSY) {
      for (int i = 0; i < 1000; i++) {
        // Hmmm.  Try resetting things.
        i2c_->CR1 |= I2C_CR1_SWRST;
      }

      for (int i = 0; i < 1000; i++) {
        i2c_->CR1 &= ~I2C_CR1_SWRST;
      }
    }
  }

  void Poll() {
    const uint16_t sr1 = i2c_->SR1;
    __attribute__((unused)) const uint16_t sr2 = i2c_->SR2;

    // Check for errors or early exits.
    switch (state_) {
      case kIdle: {
        // Just bail if we are idle.
        return;
      }
      case kDeviceAddress:
      case kReadRestart:
      case kTransferRead:
      case kTransferWrite: {
        // If we are active, first check to see if any of the error flags
        // are set.
        if ((sr1 & kSr1Errors) != 0) {
          context_.error_result =
              (static_cast<int>(state_) << 16) | (sr1 & kSr1Errors);

          i2c_->SR1 = 0;
          i2c_->CR1 |= I2C_CR1_STOP;

          state_ = kStop;
          return;
        }
        break;
      }
      case kStop: {
        // We ignore errors while waiting for a stop condition.
        break;
      }
    }

    // Now actually look for state transitions.
    switch (state_) {
      case kIdle: {
        assert(false);
        break;
      }
      case kStop: {
        // The only state we care about in the stop case is when the
        // bus is no longer busy.

        if ((i2c_->SR2 & I2C_SR2_BUSY) == 0) {
          // Great, the stop is completed.  Now we can emit our
          // callback and go back to idle.
          state_ = kIdle;
          auto callback = context_.callback;
          auto error_result = context_.error_result;
          context_ = Context();
          callback(error_result);
          return;
        }
        break;
      }
      case kDeviceAddress: {
        if (sr1 & I2C_SR1_TXE) {
          i2c_->DR = context_.memory_address;
          if (context_.tx_buffer.size()) {
            // We are writing, skip straight to the transfer phase.
            state_ = kTransferWrite;
          } else if (context_.rx_buffer.size()) {
            // Generate a restart.
            i2c_->CR1 |= I2C_CR1_START;
            state_ = kReadRestart;
          } else {
            assert(false);
          }
        }
        break;
      }
      case kReadRestart: {
        if (sr1 & I2C_SR1_SB) {
          if (context_.rx_buffer.size() == 1) {
            i2c_->CR1 &= ~I2C_CR1_ACK;
          }
          i2c_->DR = context_.device_address | 0x01;
          state_ = kTransferRead;
        }
        break;
      }
      case kTransferRead: {
        if (sr1 & I2C_SR1_RXNE) {
          if ((context_.position + 2) >=
              static_cast<int>(context_.rx_buffer.size())) {
            i2c_->CR1 &= ~I2C_CR1_ACK;
          }
          context_.rx_buffer.data()[context_.position] = i2c_->DR;
          context_.position++;
          if (context_.position >= context_.rx_buffer.size()) {
            i2c_->CR1 |= I2C_CR1_STOP;
            state_ = kStop;
          }
        }
        break;
      }
      case kTransferWrite: {
        if (sr1 & I2C_SR1_TXE) {
          if (context_.position >= context_.tx_buffer.size()) {
            i2c_->CR1 |= I2C_CR1_STOP;
            state_ = kStop;
          } else {
            i2c_->DR = context_.tx_buffer.data()[context_.position];
            context_.position++;
          }
        }
        break;
      }
    }
  }

  void AssertIdle() {
    assert(state_ == kIdle);
    assert((i2c_->SR2 & I2C_SR2_BUSY) == 0);
  }

  void SelectDevice() {
    // TODO jpieper: Possibly disable and re-enable peripheral.

    // Clear any outstanding errors.
    i2c_->SR1 &= ~(kSr1Errors);

    // Always start acknowledging.
    i2c_->CR1 |= I2C_CR1_ACK;

    // Generate the start condition. This should conclude very
    // quickly, as we don't allow multiple bus masters.
    i2c_->CR1 |= I2C_CR1_START;

    // TODO jpieper: Add minimal shortest possible timeout for this.
    while ((i2c_->SR1 & I2C_SR1_SB) == 0);

    // Start sending the address.
    i2c_->DR = context_.device_address & ~(0x01);

    state_ = kDeviceAddress;
  }

  I2C_TypeDef* const i2c_;

  enum State {
    kIdle,
    kDeviceAddress,
    kReadRestart,
    kTransferRead,
    kTransferWrite,
    kStop,
  } state_ = kIdle;

  struct Context {
    uint8_t device_address = 0;
    uint8_t memory_address = 0;
    ErrorCallback callback;
    gsl::string_span rx_buffer;
    gsl::cstring_span tx_buffer;
    uint8_t position = 0;
    int error_result = 0;
  } context_;
};

Stm32RawI2C::Stm32RawI2C(Pool& pool,
                         int i2c_number,
                         const Parameters& parameters)
: impl_(&pool, i2c_number, parameters) {}

Stm32RawI2C::~Stm32RawI2C() {}

void Stm32RawI2C::AsyncRead(uint8_t device_address,
                            uint8_t memory_address,
                            const gsl::string_span& buffer,
                            ErrorCallback callback) {
  impl_->AssertIdle();
  impl_->context_.device_address = device_address;
  impl_->context_.memory_address = memory_address;
  impl_->context_.callback = callback;
  impl_->context_.rx_buffer = buffer;

  // Generate a start condition.
  impl_->SelectDevice();
}

void Stm32RawI2C::AsyncWrite(uint8_t device_address,
                             uint8_t memory_address,
                             const gsl::cstring_span& buffer,
                             ErrorCallback callback) {
  impl_->AssertIdle();

  impl_->context_.device_address = device_address;
  impl_->context_.memory_address = memory_address;
  impl_->context_.callback = callback;
  impl_->context_.tx_buffer = buffer;

  impl_->SelectDevice();
}

void Stm32RawI2C::Poll() {
  impl_->Poll();
}
