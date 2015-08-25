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

#include <cstring>
#include <vector>

namespace legtool {
/// A very simple std::ostringstream replacement for binary data only.
class FastOStringStream {
 public:
  void write(const char* data, size_t amount) {
    std::size_t old_size = data_.size();
    data_.resize(data_.size() + amount);
    std::memcpy(&data_[old_size], data, amount);
  }

  std::string str() const { return std::string(&data_[0], data_.size()); }

 private:
  std::vector<char> data_;
};
}
