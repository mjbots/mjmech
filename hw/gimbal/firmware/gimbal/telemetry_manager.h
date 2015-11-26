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
#include "base/telemetry_archive.h"

#include "async_stream.h"
#include "pool_ptr.h"
#include "simple_stream.h"
#include "static_function.h"

class Pool;
class AsyncWriteStream;
class LockManager;

namespace detail {
template <typename T>
class ConcreteTelemetry;
}

/// The telemetry manager enables live introspection into arbitrary
/// serializable structures.
class TelemetryManager {
 public:
  TelemetryManager(Pool&, AsyncWriteStream&, LockManager&);
  ~TelemetryManager();

  /// Associate the serializable with the given name.
  ///
  /// Both @p name and @p serializable are aliased internally and must
  /// remain valid forever.
  ///
  /// @return a function which can be used to indicate that a new
  /// version of the structure is available.
  template <typename Serializable>
  StaticFunction<void ()> Register(
      const gsl::cstring_span& name, Serializable* serializable) {
    PoolPtr<detail::ConcreteTelemetry<Serializable> >
        concrete(pool(), serializable);
    return RegisterDetail(name, concrete.get());
  }

  /// This should be invoked by CommandManager.
  void Command(const gsl::cstring_span&, ErrorCallback);

  /// This should be invoked every millisecond.
  void PollMillisecond();

  /// This should be called frequently.
  void Poll();

  class Base {
   public:
    virtual ~Base() {}
    virtual int WriteBinary(OStreamInterface&) = 0;
    virtual int WriteSchema(OStreamInterface&) = 0;
  };

 private:
  StaticFunction<void ()> RegisterDetail(const gsl::cstring_span& name, Base*);

  Pool* pool() const;

  class Impl;
  PoolPtr<Impl> impl_;
};

namespace detail {
template <typename T>
class ConcreteTelemetry : public TelemetryManager::Base {
 public:
  ConcreteTelemetry(T* item) : item_(item) {}
  virtual ~ConcreteTelemetry() {}

  int WriteBinary(OStreamInterface& stream) override final {
    mjmech::base::TelemetryWriteArchive<T>::Serialize(item_, stream);
    return 0;
  }

  int WriteSchema(OStreamInterface& stream) override final {
    mjmech::base::TelemetryWriteArchive<T>::WriteSchema(stream);
    return 0;
  }

  T* item_;
};
}
