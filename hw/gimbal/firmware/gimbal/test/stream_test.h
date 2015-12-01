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
  virtual ~TestWriteStream() {}

  void AsyncWriteSome(const gsl::cstring_span& data,
                      SizeCallback callback) override final {
    // We'll only accept one character at a time to ensure that
    // retrying is working.
    ostr.write(&*data.begin(), 1);
    callback(0, 1);
  }

  std::ostringstream ostr;
};

class TestStream : public AsyncStream {
 public:
  virtual ~TestStream() {}

  void AsyncWriteSome(const gsl::cstring_span& data,
                      SizeCallback callback) override final {
    BOOST_REQUIRE_GT(data.size(), 0);
    ostr.write(&*data.begin(), data.size());
    callback(0, data.size());
  }

  void AsyncReadSome(const gsl::string_span& data,
                     SizeCallback callback) override final {
    BOOST_REQUIRE_GT(data.size(), 0);
    BOOST_REQUIRE(!read_callback_.valid());

    std::size_t to_read = std::min(data.size(), input.size());
    if (to_read == 0) {
      read_buffer_ = data;
      read_callback_ = callback;
      return;
    }

    std::memcpy(data.data(), input.data(), to_read);
    input = input.substr(to_read);

    callback(0, to_read);
  }

  void Write(const std::string& data) {
    input = input + data;

    if (read_callback_.valid()) {
      auto callback = read_callback_;
      read_callback_ = SizeCallback();
      auto buffer = read_buffer_;
      read_buffer_ = gsl::string_span();

      AsyncReadSome(buffer, callback);
    }
  }

  std::ostringstream ostr;

  std::string input;

  gsl::string_span read_buffer_;
  SizeCallback read_callback_;
};
}
