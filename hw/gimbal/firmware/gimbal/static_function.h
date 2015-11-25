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

#include <array>

#include "base/gsl/gsl-lite.h"

template <typename Signature, std::size_t Size=8>
struct StaticFunction;

template <typename R, typename... Args>
struct StaticFunctionBase {
 protected:
  struct Base {
    virtual ~Base() {}
    virtual R call(Args... args) const = 0;
    virtual void clone(void* where) const = 0;
    virtual std::size_t size() const = 0;
  };

  template <typename F>
  struct Impl : Base {
    Impl(const F& f) : f(f) {}

    R call(Args... args) const override final {
      return f(std::forward<Args>(args)...);
    }

    void clone(void* where) const override final {
      new (where) Impl<F>(*this);
    }

    std::size_t size() const override final {
      return sizeof(*this);
    }

    F f;
  };
};

/// This is like std::function, but is guaranteed to never dynamically
/// allocate memory.
template <typename R, typename ...Args, std::size_t Size>
struct StaticFunction<R(Args...), Size>
    : public StaticFunctionBase<R, Args...> {
  StaticFunction() : storage_() {}

  using Base = typename StaticFunctionBase<R, Args...>::Base;
  template <typename F>
  using Impl = typename StaticFunctionBase<R, Args...>::Impl<F>;

  template <typename F>
  StaticFunction(const F& f) {
    static_assert(alignof(Impl<F>) <= alignof(StaticFunction),
                  "alignment problem");
    static_assert(sizeof(Impl<F>) <= sizeof(storage_), "too large");

    new (storage_.data()) Impl<F>(f);
  }

  template <std::size_t OtherSize>
  StaticFunction(const StaticFunction<R(Args...), OtherSize>& rhs) {
    if (rhs.valid()) {
      Expects(rhs.getImpl().size() <= sizeof(storage_));
      rhs.getImpl().clone(data());
    }
  }

  StaticFunction(const StaticFunction& rhs) : storage_() {
    if (rhs.valid()) {
      rhs.getImpl().clone(data());
    }
  }

  StaticFunction& operator=(const StaticFunction& rhs) {
    if (valid()) { getImpl().~Base(); }
    storage_ = Storage();
    if (rhs.valid()) {
      rhs.getImpl().clone(data());
    }
    return *this;
  }

  StaticFunction(StaticFunction&& rhs) noexcept : storage_(rhs.storage_) {
    rhs.storage_ = Storage();
  }

  template <std::size_t Amount=3>
  StaticFunction<R(Args...), Size-Amount> shrink() const {
    return *this;
  }

  ~StaticFunction() {
    if (valid()) {
      getImpl().~Base();
    }
  }

  R operator()(Args... args) const {
    Expects(valid());

    return getImpl().call(std::forward<Args>(args)...);
  }

  bool valid() const { return storage_ != Storage(); }
  std::size_t size() const { return getImpl().size(); }

  const void* data() const {
    return static_cast<const void*>(storage_.data());
  }

  void* data() {
    return static_cast<void*>(storage_.data());
  }

  const Base& getImpl() const {
    Expects(valid());
    return *static_cast<const Base*>(data());
  }

  Base& getImpl() {
    Expects(valid());
    return *static_cast<Base*>(data());
  }

 private:
  typedef std::array<long, Size> Storage;
  Storage storage_;
};
