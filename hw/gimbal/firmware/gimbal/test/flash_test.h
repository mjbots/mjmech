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

#include "gimbal/flash.h"

namespace test {
class FlashTest : public FlashInterface {
 public:
  FlashTest() {}
  virtual ~FlashTest() {}

  Info GetInfo() override final {
    Info result;
    result.start = &buffer_[0];
    result.end = &buffer_[sizeof(buffer_)];
    return result;
  }

  void Erase() override final {
    std::memset(buffer_, 255, sizeof(buffer_));
    erase_count_++;
  }

  void Unlock() override final {
  }

  void Lock() override final {
  }

  void ProgramByte(char* ptr, uint8_t value) override final {
    BOOST_ASSERT(ptr >= &buffer_[0]);
    BOOST_ASSERT(ptr <= &buffer_[sizeof(buffer_)]);
    *ptr = value;
  }

 public:
  char buffer_[16384] = {};
  int erase_count_ = 0;
};
}
