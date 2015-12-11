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

class FlashInterface;

class PersistentConfig {
 public:
  PersistentConfig(Pool&, FlashInterface&);
  ~PersistentConfig();

  /// Associate the given serializable with the given name.
  ///
  /// Both @p name and @p serializable are aliased internally and must
  /// remain valid forever.
  template <typename Serializable>
  void Register(const gsl::cstring_span& name, Serializable* serializable) {
    PoolPtr<SerializableHandler<Serializable> > concrete(
        pool(), serializable);
    RegisterDetail(name, concrete.get());
  }

  /// Process the given command, responding on the given asynchronous
  /// stream, and invoking @p callback when all has been written.  It
  /// is an error to call Command while a previous instance is
  /// outstanding.
  void Command(const gsl::cstring_span&, const CommandManager::Response&);

  /// Restore all registered configuration structures from Flash.
  /// This should be invoked after all modules have had a chance to
  /// register their configurables.
  void Load();

 private:
  /// This aliases Base, which must remain valid for the lifetime of
  /// the PersistentConfig.
  void RegisterDetail(const gsl::cstring_span& name, SerializableHandlerBase*);

  Pool* pool() const;

  class Impl;
  PoolPtr<Impl> impl_;
};
