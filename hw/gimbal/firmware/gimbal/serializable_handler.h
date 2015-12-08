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

#include "async_types.h"
#include "serializable_handler_detail.h"
#include "simple_stream.h"

class SerializableHandlerBase {
 public:
  virtual ~SerializableHandlerBase() {}
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
  virtual void SetDefault() = 0;
};

template <typename T>
class SerializableHandler : public SerializableHandlerBase {
 public:
  SerializableHandler(T* item) : item_(item) {}
  virtual ~SerializableHandler() {}

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
    context->evaluate_enumerate_archive = [this, context]() {
      uint16_t current_index = 0;
      bool done = false;
      detail::EnumerateArchive(
          context, context->root_prefix,
          &current_index, &done, nullptr).Accept(this->item_);
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

  void SetDefault() override final {
    *item_ = T();
  }

 private:
  T* const item_;
};
