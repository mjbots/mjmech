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

#include "persistent_config.h"

#include <cstring>

#include "crc.h"
#include "flash.h"
#include "named_registry.h"

/// @file
///
/// The persistent storage is an unordered list of elements:
///
///   * Element
///     * pstring - name
///     * 32bit schema CRC
///     * pstring - data
///
/// It is terminated by an element with a 0 length name.
///
/// A pstring is a 32 byte unsigned integer followed by that many
/// bytes of data.


class PersistentConfig::Impl {
 public:
  Impl(Pool& pool, FlashInterface& flash, AsyncWriteStream& stream)
      : pool_(pool), flash_(flash), stream_(stream) {}

  void Enumerate(ErrorCallback callback) {
    current_callback_ = callback;
    current_enumerate_index_ = 0;

    EnumerateCallback(0);
  }

  void EnumerateCallback(int error) {
    if (error) { current_callback_(error); return; }

    if (current_enumerate_index_ >= elements_.size()) {
      WriteOK(current_callback_);
      return;
    }

    auto* const element = &elements_[current_enumerate_index_];
    if (element->name.size() == 0) {
      WriteOK(current_callback_);
      return;
    }

    current_enumerate_index_++;

    element->ptr->Enumerate(&this->enumerate_context_,
                            this->send_buffer_,
                            element->name,
                            stream_,
                            [this](int err) { this->EnumerateCallback(err); });
  }

  void Get(const gsl::cstring_span& field,
           ErrorCallback callback) {
    Tokenizer tokenizer(field, ".");
    auto group = tokenizer.next();
    auto* const element =
        elements_.FindOrCreate(group, NamedRegistry::kFindOnly);
    if (element == nullptr) {
      WriteMessage(gsl::ensure_z("unknown group\r\n"), callback);
    } else {
      current_callback_ = callback;
      int err =
          element->ptr->Read(
              send_buffer_, tokenizer.remaining(), stream_,
              [this](int error) {
                if (error) { this->current_callback_(error); return; }
                WriteMessage(gsl::ensure_z("\r\n"),
                             this->current_callback_);
              });
      if (err) {
        WriteMessage(gsl::ensure_z("error reading\r\n"), callback);
      }
    }
  }

  void Set(const gsl::cstring_span& command,
           ErrorCallback callback) {
    Tokenizer tokenizer(command, ".");
    auto group = tokenizer.next();
    auto* const element =
        elements_.FindOrCreate(group, NamedRegistry::kFindOnly);
    if (element == nullptr) {
      WriteMessage(gsl::ensure_z("unknown group\r\n"), callback);
    } else {
      Tokenizer name_value(tokenizer.remaining(), " ");
      auto key = name_value.next();
      auto value = name_value.remaining();
      int result = element->ptr->Set(key, value);
      if (result == 0) {
        WriteOK(callback);
      } else {
        WriteMessage(gsl::ensure_z("error setting\r\n"), callback);
      }
    }
  }

  void Load(ErrorCallback callback) {
    auto info = flash_.GetInfo();
    SimpleIStream flash_stream(info.start, info.end - info.start);
    mjmech::base::TelemetryReadStream<SimpleIStream> stream(flash_stream);

    while (true) {
      uint32_t name_size = stream.Read<uint32_t>();
      if (name_size == 0) { break; }
      gsl::cstring_span name(flash_stream.current(),
                             flash_stream.current() + name_size);
      flash_stream.ignore(name_size);

      const uint32_t expected_crc = stream.Read<uint32_t>();
      const uint32_t data_size = stream.Read<uint32_t>();

      // We are now committed to reading the entirety of the data one
      // way or another.

      auto* const element =
          elements_.FindOrCreate(name, NamedRegistry::kFindOnly);
      if (element == nullptr) {
        // TODO jpieper: It would be nice to warn about situations
        // like this.
        flash_stream.ignore(data_size);
        continue;
      }

      const uint32_t actual_crc = CalculateSchemaCrc(element->ptr);
      if (actual_crc != expected_crc) {
        // TODO jpieper: It would be nice to warn about situations like
        // this.
        flash_stream.ignore(data_size);
        continue;
      }

      element->ptr->ReadBinary(flash_stream);
    }

    WriteOK(callback);
  }

  uint32_t CalculateSchemaCrc(SerializableHandlerBase* base) const {
    char schema_buffer[2048] = {};
    SimpleOStream schema_stream(schema_buffer, sizeof(schema_buffer));
    base->WriteSchema(schema_stream);

    // Calculate CRC of schema.
    const uint32_t crc = CalculateCrc(schema_buffer, schema_stream.position());

    return crc;
  }

  void Write(ErrorCallback callback) {
    auto info = flash_.GetInfo();
    flash_.Unlock();
    flash_.Erase();
    FlashWriteStream flash_stream(flash_, info.start);
    mjmech::base::TelemetryWriteStream<FlashWriteStream> stream(flash_stream);

    for (size_t i = 0; i < elements_.size(); i++) {
      if (elements_[i].name.size() == 0) { break; }
      stream.Write(elements_[i].name);
      stream.Write(static_cast<uint32_t>(CalculateSchemaCrc(elements_[i].ptr)));

      char* const data_size_position = flash_stream.position();
      flash_stream.skip(sizeof(uint32_t)); // size
      char* const data_start = flash_stream.position();
      elements_[i].ptr->WriteBinary(flash_stream);

      const uint32_t data_size = flash_stream.position() - data_start;
      {
        FlashWriteStream data_size_stream(flash_, data_size_position);
        mjmech::base::TelemetryWriteStream<FlashWriteStream> s(
            data_size_stream);
        s.Write(data_size);
      }
    }

    stream.Write(static_cast<uint32_t>(0));

    flash_.Lock();

    WriteOK(callback);
  }

  void WriteOK(ErrorCallback callback) {
    WriteMessage(gsl::ensure_z("OK\r\n"), callback);
  }

  void UnknownCommand(const gsl::cstring_span& command,
                      ErrorCallback callback) {
    WriteMessage(gsl::ensure_z("unknown command\r\n"), callback);
  }

  void WriteMessage(const gsl::cstring_span& message,
                    ErrorCallback callback) {
    AsyncWrite(stream_, message, callback);
  }

  Pool& pool_;
  FlashInterface& flash_;
  AsyncWriteStream& stream_;

  typedef NamedRegistryBase<SerializableHandlerBase, 8> NamedRegistry;
  NamedRegistry elements_;

  // TODO jpieper: This buffer could be shared with other things that
  // have the same output stream, as only one should be writing at a
  // time anyways.
  char send_buffer_[256] = {};

  ErrorCallback current_callback_;
  std::size_t current_enumerate_index_ = 0;
  detail::EnumerateArchive::Context enumerate_context_;
};

PersistentConfig::PersistentConfig(
    Pool& pool, FlashInterface& flash, AsyncWriteStream& stream)
    : impl_(&pool, pool, flash, stream) {
}

PersistentConfig::~PersistentConfig() {
}

void PersistentConfig::Command(const gsl::cstring_span& command,
                               ErrorCallback callback) {
  Tokenizer tokenizer(command, " ");
  auto cmd = tokenizer.next();
  if (cmd == gsl::ensure_z("enumerate")) {
    impl_->Enumerate(callback);
  } else if (cmd == gsl::ensure_z("get")) {
    impl_->Get(tokenizer.remaining(), callback);
  } else if (cmd == gsl::ensure_z("set")) {
    impl_->Set(tokenizer.remaining(), callback);
  } else if (cmd == gsl::ensure_z("load")) {
    impl_->Load(callback);
  } else if (cmd == gsl::ensure_z("write")) {
    impl_->Write(callback);
  } else {
    impl_->UnknownCommand(cmd, callback);
  }
}

void PersistentConfig::Load() {
}

void PersistentConfig::RegisterDetail(
    const gsl::cstring_span& name, SerializableHandlerBase* base) {
  auto* const element = impl_->elements_.FindOrCreate(
      name, Impl::NamedRegistry::kAllowCreate);
  element->ptr = base;
}

Pool* PersistentConfig::pool() const { return &impl_->pool_; }
