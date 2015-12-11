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
#include "command_manager.h"
#include "pool_ptr.h"
#include "serializable_handler.h"
#include "simple_stream.h"
#include "static_function.h"

class Pool;
class AsyncWriteStream;
class LockManager;

/// The telemetry manager enables live introspection into arbitrary
/// serializable structures.
class TelemetryManager {
 public:
  TelemetryManager(Pool&, LockManager&);
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
    PoolPtr<SerializableHandler<Serializable> >
        concrete(pool(), serializable);
    return RegisterDetail(name, concrete.get());
  }

  /// This should be invoked by CommandManager.
  void Command(const gsl::cstring_span&, const CommandManager::Response&);

  /// This should be invoked every millisecond.
  void PollMillisecond();

  /// This should be called frequently.
  void Poll();

 private:
  StaticFunction<void ()> RegisterDetail(
      const gsl::cstring_span& name, SerializableHandlerBase*);

  Pool* pool() const;

  class Impl;
  PoolPtr<Impl> impl_;
};
