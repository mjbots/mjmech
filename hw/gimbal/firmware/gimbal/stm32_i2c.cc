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

#include "stm32_i2c.h"

#include <assert.h>

namespace {
struct Registry {
  I2C_HandleTypeDef* hi2c = nullptr;
  Stm32I2C* stm32 = nullptr;
};

Registry g_registry[3] = {};
}

extern "C" {

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  for (auto& item: g_registry) {
    if (item.hi2c == hi2c) {
      item.stm32->TransmitComplete();
      return;
    }
  }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  for (auto& item: g_registry) {
    if (item.hi2c == hi2c) {
      item.stm32->ReceiveComplete();
      return;
    }
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  for (auto& item: g_registry) {
    if (item.hi2c == hi2c) {
      item.stm32->Error();
      return;
    }
  }
}
}

Stm32I2C::Stm32I2C(I2C_HandleTypeDef* hi2c) : hi2c_(hi2c) {
  for (auto& item: g_registry) {
    if (item.hi2c == nullptr) {
      item.hi2c = hi2c;
      item.stm32 = this;
      return;
    }
  }
  assert(false);
}

Stm32I2C::~Stm32I2C() {
  for (auto& item: g_registry) {
    if (item.hi2c == hi2c_) {
      assert(item.stm32 == this);
      item.hi2c = nullptr;
      item.stm32 = nullptr;
      return;
    }
  }
}

void Stm32I2C::AsyncRead(uint8_t device_address,
                         uint8_t memory_address,
                         const gsl::string_span& buffer,
                         ErrorCallback callback) {
  assert(!read_callback_.valid());
  assert(!write_callback_.valid());

  read_callback_ = callback;

  auto i2c_status = HAL_I2C_Mem_Read_DMA(
      hi2c_, device_address, memory_address, I2C_MEMADD_SIZE_8BIT,
      reinterpret_cast<uint8_t*>(buffer.data()), buffer.size());

  if (i2c_status != 0) {
    read_callback_ = ErrorCallback();
    callback(i2c_status);
  }
}

void Stm32I2C::AsyncWrite(uint8_t device_address,
                          uint8_t memory_address,
                          const gsl::cstring_span& buffer,
                          ErrorCallback callback) {
  assert(!read_callback_.valid());
  assert(!write_callback_.valid());

  write_callback_ = callback;

  auto i2c_status = HAL_I2C_Mem_Write_DMA(
      hi2c_, device_address, memory_address, I2C_MEMADD_SIZE_8BIT,
      const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(buffer.data())),
      buffer.size());

  if (i2c_status != 0) {
    write_callback_ = ErrorCallback();
    callback(i2c_status);
  }
}

void Stm32I2C::ReceiveComplete() {
  if (!read_callback_.valid()) { return; }

  auto callback = read_callback_;
  read_callback_ = ErrorCallback();
  callback(0);
}

void Stm32I2C::TransmitComplete() {
  if (!write_callback_.valid()) { return; }

  auto callback = write_callback_;
  write_callback_ = ErrorCallback();
  callback(0);
}

void Stm32I2C::Error() {
  if (!write_callback_.valid() &&
      !read_callback_.valid()) { return; }

  assert(write_callback_.valid() ^ read_callback_.valid());

  if (write_callback_.valid()) {
    auto callback = write_callback_;
    write_callback_ = ErrorCallback();
    callback(0x1000 | hi2c_->ErrorCode);
  } else if (read_callback_.valid()) {
    auto callback = read_callback_;
    read_callback_ = ErrorCallback();
    callback(0x1000 | hi2c_->ErrorCode);
  }
}
