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

#include <set>

#include "gimbal/async_i2c.h"

namespace test {
class AsyncI2CSimulator : public AsyncI2C {
 public:
  AsyncI2CSimulator(uint8_t address) : address_(address) {}

  virtual void AsyncRead(uint8_t device_address,
                         uint8_t memory_address,
                         const gsl::string_span& buffer,
                         ErrorCallback callback) override {
    BOOST_CHECK_EQUAL(device_address, address_);

    for (std::size_t i = 0; i < buffer.size(); i++) {
      addresses_read_.insert(memory_address + i);
      *(buffer.data() + i) = static_cast<char>(Read(memory_address + i));
    }

    read_callback_ = callback;
  }

  virtual void AsyncWrite(uint8_t device_address,
                          uint8_t memory_address,
                          const gsl::cstring_span& buffer,
                          ErrorCallback callback) override {
    BOOST_CHECK_EQUAL(device_address, address_);

    for (std::size_t i = 0; i < buffer.size(); i++) {
      addresses_written_.insert(memory_address + i);
      Write(memory_address + i, static_cast<uint8_t>(*(buffer.data() + i)));
    }

    write_callback_ = callback;
  }

  virtual uint8_t Read(uint8_t address) = 0;
  virtual void Write(uint8_t address, uint8_t value) = 0;

  bool ProcessAction() {
    if (!write_callback_.valid() &&
        !read_callback_.valid()) {
      return false;
    }

    BOOST_CHECK_EQUAL(write_callback_.valid() ^ read_callback_.valid(), true);

    if (read_callback_.valid()) {
      auto callback = read_callback_;
      read_callback_ = ErrorCallback();
      callback(0);
    } else {
      auto callback = write_callback_;
      write_callback_ = ErrorCallback();
      callback(0);
    }

    return true;
  }

  const uint8_t address_;
  ErrorCallback read_callback_;
  ErrorCallback write_callback_;
  std::set<int> addresses_read_;
  std::set<int> addresses_written_;
};
}
