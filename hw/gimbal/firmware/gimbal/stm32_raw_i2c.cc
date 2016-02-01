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
#include "stm32f4xx_hal_rcc.h"
#include "gpio.h"

#include "clock.h"
#include "gpio_pin.h"

// TODO jpieper: Use DMA for the bulk transfers to reduce overhead.

namespace {
typedef Stm32RawI2C::Parameters Parameters;

const int kResetTimeoutCount = 1000;
const int kTransferTimeoutMilliseconds = 2;
const int kErrorTimeoutMilliseconds = 1;

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

template <typename Predicate>
bool TestTimeout(Predicate p) {
  for (int i = 0; i < kResetTimeoutCount; i++) {
    if (p()) { return true; }
  }
  return false;
}
}

class Stm32RawI2C::Impl {
 public:
  Impl(int i2c_number, GpioPin& scl, GpioPin& sda,
       const Parameters& parameters, Clock& clock)
      : i2c_(GetI2C(i2c_number)), scl_(scl), sda_(sda),
        parameters_(parameters), clock_(clock) {
    assert(i2c_number >= 1 && i2c_number <= 3);

    Init();
  }

  void Init() {
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t freqrange = I2C_FREQRANGE(pclk1);

    // Start out with the peripheral disabled.
    i2c_->CR1 = 0;

    i2c_->CR2 = freqrange;

    i2c_->TRISE = (parameters_.speed <= 100000) ?
        (freqrange + 1) :
        (freqrange * 300 / 1000 + 1);

    i2c_->CCR = CalculateCCR(pclk1, parameters_.speed, parameters_.duty_cycle);

    // Start out enabled, with no fancy options.
    i2c_->CR1 = I2C_CR1_PE;

    // This class does master only, so we don't care about our own
    // addresses.
    i2c_->OAR1 = 0;
    i2c_->OAR2 = 0;
  }

  void ResetBus() {
    // NOTE jpieper: This reset procedure has been very carefully
    // crafted based on single stepping while examining the device
    // registers under a number of possible failure scenarios.  The
    // I2C hardware is pretty finicky, and is hard to get back to a
    // known state, and also external devices may be in a crappy state
    // as well.
    //
    // Before changing this, be ready to do some careful work, and be
    // sure to at least test resetting in the middle of a transaction
    // when SCL is free, and when SCL is held low by a slave.

    // Start with the peripheral disabled.
    i2c_->CR1 &= ~I2C_CR1_PE;

    // Do a pre-emptory reset of the peripheral.  This wipes all
    // config registers and releases the bus, although any slaves that
    // have things down will still do so.
    i2c_->CR1 |= I2C_CR1_SWRST;
    i2c_->CR1 &= ~I2C_CR1_SWRST;

    const auto set_pin = [](GpioPin& pin, bool value) {
      pin.Set(value);
      TestTimeout([&]() { return pin.Read() == value; });
    };

    set_pin(scl_, true);
    set_pin(sda_, true);

    // Switching the device into output mode will cause the I2C
    // peripheral to report busy, even if the pins did not change
    // state.
    scl_.SetMode(GpioPin::kGeneralOutput);
    sda_.SetMode(GpioPin::kGeneralOutput);

    // Now do stuff to try and unstick external peripherals.

    if (!scl_.Read()) {
      // Try to get SCL unstuck.
      for (int i = 0; i < 10; i++) {
        set_pin(sda_, false);
        set_pin(sda_, true);
        if (scl_.Read()) { break; }
      }
    }

    // Then generate a start and stop condition.
    set_pin(sda_, false);
    set_pin(scl_, false);
    set_pin(scl_, true);
    set_pin(sda_, true);

    // Finally, go back into AF mode (which should cause no external
    // pin change.
    sda_.SetMode(GpioPin::kAlternateFunction);
    scl_.SetMode(GpioPin::kAlternateFunction);

    // Do a final reset to clear the busy flag that our above regular
    // output mode twiddling induced.
    i2c_->CR1 |= I2C_CR1_SWRST;
    i2c_->CR1 &= ~I2C_CR1_SWRST;

    // Now that we're good to go, reconfigure the I2C peripheral's
    // registers.
    Init();
 }

  void Poll() {
    const uint16_t sr1 = i2c_->SR1;
    __attribute__((unused)) const uint16_t sr2 = i2c_->SR2;

    assert(i2c_->CR1 & I2C_CR1_PE);

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
          if (!(sr1 & I2C_SR1_ARLO) &&
              (sr2 & I2C_SR2_BUSY) &&
              (sr2 & I2C_SR2_MSL)) {
            i2c_->CR1 |= I2C_CR1_STOP;
          }

          state_ = kStop;
          return;
        }
        break;
      }
      case kStop: {
        // We ignore errors while waiting for a stop condition.
        break;
      }
      case kError: {
        const uint32_t now = clock_.timestamp();
        const uint32_t delta = now - start_time_;
        if (delta > (kErrorTimeoutMilliseconds * 10)) {
          // Add an extra flag to indicate we had to try a bus reset.
          context_.error_result |= (0x100 << 16);
          ResetBus();
          start_time_ = now;
          return;
        }

        if (i2c_->CR1 & I2C_CR1_ACK) {
          // We are already receiving things.
          i2c_->CR1 &= ~I2C_CR1_ACK;
          need_to_read_ = (sr1 & I2C_SR1_BTF) ? 2 : 1;
          return;
        }

        if ((sr2 & I2C_SR2_BUSY) &&
            (sr2 & I2C_SR2_MSL) &&
            !(sr1 & I2C_SR1_STOPF) &&
            !(i2c_->CR1 & I2C_CR1_STOP) &&
            need_to_read_ == 0) {
          i2c_->CR1 |= I2C_CR1_STOP;
          return;
        }

        // We can't return until we have gotten the peripheral into a
        // known state.
        if (sr1 & I2C_SR1_RXNE) {
          // Burn the data.
          __attribute__((unused)) uint8_t data = i2c_->DR;
          if (need_to_read_ > 0) { need_to_read_--; }
          return;
        }

        if (sr2 & I2C_SR2_BUSY) {
          // Wait out a busy signal, hoping our above actions will
          // resolve it.
          return;
        }

        i2c_->SR1 = 0;
        state_ = kIdle;
        auto callback = context_.callback;
        auto error_result = context_.error_result;
        context_ = Context();
        callback(error_result);
        return;
      }
    }

    // Now actually look for state transitions.
    switch (state_) {
      case kIdle:
      case kError: {
        assert(false);
        break;
      }
      case kStop: {
        // The only state we care about in the stop case is when the
        // bus is no longer busy.
        if ((sr2 & I2C_SR2_BUSY) == 0) {
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

    // Finally, check for timeouts.
    switch (state_) {
      case kIdle:
      case kError: {
        assert(false);
        break;
      }
      case kDeviceAddress:
      case kReadRestart:
      case kTransferRead:
      case kTransferWrite:
      case kStop: {
        const uint32_t now = clock_.timestamp();
        const uint32_t delta = now - start_time_;
        if (delta > (kTransferTimeoutMilliseconds * 10)) {
          context_.error_result = (18 << 16) | i2c_->SR1;
          state_ = kError;
          need_to_read_ = 0;
          start_time_ = now;
        }
        break;
      }
    }
  }

  void AssertIdle() {
    assert(state_ == kIdle);
  }

  void SelectDevice() {
    const uint16_t sr1 = i2c_->SR1;
    const uint16_t sr2 = i2c_->SR2;

    if (sr2 & I2C_SR2_BUSY) {
      // Hmph, there must have been noise on the line or something.
      // Let our normal error handling deal with it.
      context_.error_result = (16 << 16) | sr1;
      state_ = kError;
      need_to_read_ = 0;
      start_time_ = clock_.timestamp();
      return;
    }

    // Clear any outstanding errors.
    i2c_->SR1 &= ~(kSr1Errors);

    // Always start acknowledging.
    i2c_->CR1 |= I2C_CR1_ACK;

    // Generate the start condition. This should conclude very
    // quickly, as we don't allow multiple bus masters.
    i2c_->CR1 |= I2C_CR1_START;

    // Wait only a short while for the start bit to be set.
    const bool start_bit =
        TestTimeout([&](){ return (i2c_->SR1 & I2C_SR1_SB) != 0; });
    if (!start_bit) {
      // Report an error.  No need to send a stop bit, as we never
      // sent a start.
      context_.error_result = (17 << 16) | i2c_->SR1;
      state_ = kError;
      need_to_read_ = 0;
      start_time_ = clock_.timestamp();
      return;
    }

    // Start sending the address.
    i2c_->DR = context_.device_address & ~(0x01);

    start_time_ = clock_.timestamp();
    state_ = kDeviceAddress;
  }

  I2C_TypeDef* const i2c_;
  GpioPin& scl_;
  GpioPin& sda_;
  const Parameters parameters_;
  Clock& clock_;
  uint32_t start_time_ = 0;
  uint8_t need_to_read_ = 0;

  enum State {
    kIdle,
    kDeviceAddress,
    kReadRestart,
    kTransferRead,
    kTransferWrite,
    kStop,
    kError,
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
                         GpioPin& scl,
                         GpioPin& sda,
                         const Parameters& parameters,
                         Clock& clock)
: impl_(&pool, i2c_number, scl, sda, parameters, clock) {}

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
