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

#include "base/gsl/gsl-lite.h"

/// A dumb circular buffer which works with move-only objects.
template <typename T, std::size_t Size>
class circular_buffer {
 public:
  circular_buffer() {}

  void push_back(T&& value) {
    Expects(!full());
    data_[insert_] = std::move(value);
    insert_ = (insert_ + 1) % Size;
  }

  void pop_front() {
    Expects(!empty());
    remove_ = (remove_ + 1) % Size;
  }

  T front() const { return data_[remove_]; }
  T back() const { return data_[insert_]; }

  bool empty() const { return insert_ == remove_; }
  bool full() const {
    return ((insert_ + 1) % Size) == remove_;
  }

  size_t capacity() const { return data_.size() - 1; }

  size_t size() const {
    return (insert_ >= remove_) ?
        (insert_ - remove_) :
        (insert_ + Size - remove_);
  }

 private:
  volatile T data_[Size] = {};
  volatile size_t insert_ = 0;
  volatile size_t remove_ = 0;
};
