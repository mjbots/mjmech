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

#include <cstddef>
#include <utility>

/// Provides access to unformatted data.
class Pool {
 public:
  Pool(char* data, std::size_t size) : data_(data), size_(size) {}

  void* Allocate(std::size_t size, std::size_t alignment);

  std::size_t size() const { return size_; }
  std::size_t available() const { return size_ - position_; }

 private:
  char* const data_;
  const std::size_t size_;

  std::size_t position_ = 0;
};

/// This is a concrete pool which has a user-defined size.
template <size_t N=8192>
class SizedPool : public Pool {
 public:
  SizedPool() : Pool(data_, N) {}

 private:
  char data_[N] = {};
};

/// A simple smart pointer which allocates from a fixed size pool and
/// never de-allocates.
template <typename T>
class PoolPtr {
 public:
  enum {
    kAlignment = 8,
  };

  template <typename... Args>
  PoolPtr(Pool* pool, Args&&... args)
      : ptr_(reinterpret_cast<T*>(pool->Allocate(sizeof(T), alignof(T)))) {
    new (ptr_) T(std::forward<Args>(args)...);
  }

  ~PoolPtr() {
    // This class should not be used with things that need their
    // destructors to be called.
  }

  // Accessors to appear like a regular pointer.

  T& operator*() { return *ptr_; }
  const T& operator*() const { return *ptr_; }
  T* operator->() { return ptr_; }
  const T* operator->() const { return ptr_; }

  // And accessors to appear like a smart pointer.

  T* get() { return ptr_; }
  const T* get() const { return ptr_; }

 private:
  T* const ptr_;
};
