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

#include "gimbal/async_stream.h"

namespace test {
class TestWriteStream : public AsyncWriteStream {
 public:
  ~TestWriteStream() override {}

  void AsyncWriteSome(const gsl::cstring_span& data,
                      SizeCallback callback) override final {
    // We'll only accept one character at a time to ensure that
    // retrying is working.
    ostr.write(&*data.begin(), 1);
    callback(0, 1);
  }

  std::ostringstream ostr;
};
}
