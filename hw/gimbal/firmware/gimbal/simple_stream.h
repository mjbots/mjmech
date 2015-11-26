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

class OStreamInterface {
 public:
  virtual void write(const char* data, size_t amount) = 0;
};

class SimpleOStream : public OStreamInterface {
 public:
  SimpleOStream(char* buffer, std::size_t size)
      : data_(buffer), size_(size), position_(0) {}

  void write(const char* data, size_t amount) override {
    Expects(position_ + amount <= size_);

    std::memcpy(&data_[position_], data, amount);
    position_ += amount;
  }

  void write(const gsl::cstring_span& data) {
    write(data.data(), data.size());
  }

  void skip(std::size_t size) {
    position_ += size;
  }

  std::size_t position() const { return position_; }

 private:
  char* const data_;
  const std::size_t size_;
  size_t position_;
};

class IStreamInterface {
 public:
  virtual void ignore(size_t length) = 0;
  virtual void read(char* buffer, size_t length) = 0;
  virtual std::size_t gcount() const = 0;
};

class SimpleIStream : public IStreamInterface {
 public:
  SimpleIStream(const char* buffer, std::size_t size)
      :data_(buffer), size_(size), position_(0), last_read_(0) {}

  void ignore(size_t length) override {
    Expects(position_ + length <= size_);
    position_ += length;
  }

  void read(char* buffer, size_t length) override {
    last_read_ = std::min(size_ - position_, length);
    std::memcpy(buffer, &data_[position_], last_read_);
    position_ += last_read_;
  }

  std::size_t gcount() const override { return last_read_; }

  const char* current() const { return &data_[position_]; }

 private:
  const char* const data_;
  const std::size_t size_;
  std::size_t position_;
  std::size_t last_read_;
};
