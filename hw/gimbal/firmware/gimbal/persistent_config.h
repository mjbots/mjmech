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
#include "persistent_config_detail.h"
#include "pool_ptr.h"
#include "simple_stream.h"

namespace detail {
template <typename T>
class ConcreteBase;
}

class FlashInterface;

class PersistentConfig {
 public:
  PersistentConfig(Pool&, FlashInterface&, AsyncWriteStream&);
  ~PersistentConfig();

  /// Associate the given serializable with the given name.
  ///
  /// Both @p name and @p serializable are aliased internally and must
  /// remain valid forever.
  template <typename Serializable>
  void Register(const gsl::cstring_span& name, Serializable* serializable) {
    PoolPtr<detail::ConcreteBase<Serializable> > concrete(pool(), serializable);
    RegisterDetail(name, concrete.get());
  }

  /// Process the given command, responding on the given asynchronous
  /// stream, and invoking @p callback when all has been written.  It
  /// is an error to call Command while a previous instance is
  /// outstanding.
  void Command(const gsl::cstring_span&, ErrorCallback);

  /// Restore all registered configuration structures from Flash.
  /// This should be invoked after all modules have had a chance to
  /// register their configurables.
  void Load();

  class Base {
   public:
    virtual ~Base() {}
    virtual int WriteBinary(OStreamInterface&) = 0;
    virtual void WriteSchema(OStreamInterface&) = 0;
    virtual int ReadBinary(IStreamInterface&) = 0;
    virtual int Set(const gsl::cstring_span&, const gsl::cstring_span&) = 0;
    virtual void Enumerate(detail::EnumerateArchive::Context*,
                           const gsl::string_span& buffer,
                           const gsl::cstring_span& prefix,
                           AsyncWriteStream&,
                           ErrorCallback) = 0;
    virtual int Read(const gsl::string_span& buffer,
                     const gsl::cstring_span& key,
                     AsyncWriteStream&,
                     ErrorCallback) = 0;
  };

 private:
  /// This aliases Base, which must remain valid for the lifetime of
  /// the PersistentConfig.
  void RegisterDetail(const gsl::cstring_span& name, Base*);

  Pool* pool() const;

  class Impl;
  PoolPtr<Impl> impl_;
};

namespace detail {
template <typename T>
class ConcreteBase : public PersistentConfig::Base {
 public:
  ConcreteBase(T* item) : item_(item) {}
  virtual ~ConcreteBase() {}

  int WriteBinary(OStreamInterface& stream) override final {
    mjmech::base::TelemetryWriteArchive<T>::Serialize(item_, stream);
    return 0;
  }

  void WriteSchema(OStreamInterface& stream) override final {
    mjmech::base::TelemetryWriteArchive<T>::WriteSchema(stream);
  }

  int ReadBinary(IStreamInterface& stream) override final {
    mjmech::base::TelemetrySimpleReadArchive<T>::Deserialize(item_, stream);
    return 0;
  }

  int Set(const gsl::cstring_span& key,
          const gsl::cstring_span& value) override final {
    detail::SetArchive(key, value).Accept(item_);
    return 0;
  }

  void Enumerate(detail::EnumerateArchive::Context* context,
                 const gsl::string_span& buffer,
                 const gsl::cstring_span& prefix,
                 AsyncWriteStream& stream,
                 ErrorCallback callback) override final {
    context->root_prefix = prefix;
    context->stream = &stream;
    context->buffer = buffer;
    context->callback = callback;
    context->current_field_index_to_write = 0;
    context->evaluate_enumerate_archive = [&]() {
      uint16_t current_index = 0;
      bool done = false;
      detail::EnumerateArchive(context, context->root_prefix,
                               &current_index, &done, nullptr).Accept(item_);
      return done;
    };

    context->evaluate_enumerate_archive();
  }

  int Read(const gsl::string_span& buffer,
           const gsl::cstring_span& key,
           AsyncWriteStream& stream,
           ErrorCallback callback) override final {
    detail::ReadArchive archive(key, buffer, stream, callback);
    archive.Accept(item_);
    return archive.found() ? 0 : 1;
  }

 private:
  T* const item_;
};
}
