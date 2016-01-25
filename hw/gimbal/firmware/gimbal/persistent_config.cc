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
  Impl(Pool& pool, FlashInterface& flash)
      : pool_(pool), flash_(flash) {}

  struct Element {
    SerializableHandlerBase* serializable = nullptr;
    StaticFunction<void ()> updated;
  };

  typedef NamedRegistryBase<Element, 16> NamedRegistry;

  void Enumerate(const CommandManager::Response& response) {
    current_response_ = response;
    current_enumerate_index_ = 0;

    EnumerateCallback(0);
  }

  void EnumerateCallback(int error) {
    if (error) { current_response_.callback(error); return; }

    if (current_enumerate_index_ >= elements_.size()) {
      WriteOK(current_response_);
      return;
    }

    auto* const element = &elements_[current_enumerate_index_];
    if (element->name.size() == 0) {
      WriteOK(current_response_);
      return;
    }

    current_enumerate_index_++;

    element->ptr->serializable->Enumerate(
        &this->enumerate_context_,
        this->send_buffer_,
        element->name,
        *current_response_.stream,
        [this](int err) { this->EnumerateCallback(err); });
  }

  void Get(const gsl::cstring_span& field,
           const CommandManager::Response& response) {
    Tokenizer tokenizer(field, ".");
    auto group = tokenizer.next();
    auto* const element =
        elements_.FindOrCreate(group, NamedRegistry::kFindOnly);
    if (element == nullptr) {
      WriteMessage(gsl::ensure_z("unknown group\r\n"), response);
    } else {
      current_response_ = response;
      int err =
          element->ptr->serializable->Read(
              send_buffer_,
              tokenizer.remaining(),
              *current_response_.stream,
              [this](int error) {
                if (error) {
                  this->current_response_.callback(error);
                  return;
                }
                WriteMessage(gsl::ensure_z("\r\n"),
                             this->current_response_);
              });
      if (err) {
        WriteMessage(gsl::ensure_z("error reading\r\n"), response);
      }
    }
  }

  void Set(const gsl::cstring_span& command,
           const CommandManager::Response& response) {
    Tokenizer tokenizer(command, ".");
    auto group = tokenizer.next();
    auto* const element =
        elements_.FindOrCreate(group, NamedRegistry::kFindOnly);
    if (element == nullptr) {
      WriteMessage(gsl::ensure_z("unknown group\r\n"), response);
    } else {
      Tokenizer name_value(tokenizer.remaining(), " ");
      auto key = name_value.next();
      auto value = name_value.remaining();
      int result = element->ptr->serializable->Set(key, value);
      if (result == 0) {
        element->ptr->updated();
        WriteOK(response);
      } else {
        WriteMessage(gsl::ensure_z("error setting\r\n"), response);
      }
    }
  }

  void Load(const CommandManager::Response& response) {
    DoLoad();

    WriteOK(response);
  }

  void DoLoad() {
    auto info = flash_.GetInfo();
    SimpleIStream flash_stream(info.start, info.end - info.start);
    mjmech::base::TelemetryReadStream<SimpleIStream> stream(flash_stream);

    while (true) {
      uint32_t name_size = stream.Read<uint32_t>();
      typedef mjmech::base::TelemetryFormat TF;
      if (name_size == 0 ||
          name_size >= static_cast<uint32_t>(TF::BlockOffsets::kMaxBlockSize)) {
        break;
      }
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

      const uint32_t actual_crc = CalculateSchemaCrc(element->ptr->serializable);
      if (actual_crc != expected_crc) {
        // TODO jpieper: It would be nice to warn about situations like
        // this.
        flash_stream.ignore(data_size);
        continue;
      }

      element->ptr->serializable->ReadBinary(flash_stream);
    }
  }

  uint32_t CalculateSchemaCrc(SerializableHandlerBase* base) const {
    char schema_buffer[2048] = {};
    SimpleOStream schema_stream(schema_buffer, sizeof(schema_buffer));
    base->WriteSchema(schema_stream);

    // Calculate CRC of schema.
    const uint32_t crc = CalculateCrc(schema_buffer, schema_stream.position());

    return crc;
  }

  void Write(const CommandManager::Response& response) {
    auto info = flash_.GetInfo();
    flash_.Unlock();
    flash_.Erase();
    FlashWriteStream flash_stream(flash_, info.start);
    mjmech::base::TelemetryWriteStream<FlashWriteStream> stream(flash_stream);

    for (size_t i = 0; i < elements_.size(); i++) {
      if (elements_[i].name.size() == 0) { break; }
      stream.Write(elements_[i].name);
      stream.Write(static_cast<uint32_t>(
                       CalculateSchemaCrc(elements_[i].ptr->serializable)));

      char* const data_size_position = flash_stream.position();
      flash_stream.skip(sizeof(uint32_t)); // size
      char* const data_start = flash_stream.position();
      elements_[i].ptr->serializable->WriteBinary(flash_stream);

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

    WriteOK(response);
  }

  void Default(const CommandManager::Response& response) {
    for (size_t i = 0; i < elements_.size(); i++) {
      if (elements_[i].name.size() == 0) { break; }
      elements_[i].ptr->serializable->SetDefault();
    }
    WriteOK(response);
  }

  void WriteOK(const CommandManager::Response& response) {
    WriteMessage(gsl::ensure_z("OK\r\n"), response);
  }

  void UnknownCommand(const gsl::cstring_span& command,
                      const CommandManager::Response& response) {
    WriteMessage(gsl::ensure_z("unknown command\r\n"), response);
  }

  void WriteMessage(const gsl::cstring_span& message,
                    const CommandManager::Response& response) {
    AsyncWrite(*response.stream, message, response.callback);
  }

  Pool& pool_;
  FlashInterface& flash_;

  NamedRegistry elements_;

  // TODO jpieper: This buffer could be shared with other things that
  // have the same output stream, as only one should be writing at a
  // time anyways.
  char send_buffer_[256] = {};

  CommandManager::Response current_response_;
  std::size_t current_enumerate_index_ = 0;
  detail::EnumerateArchive::Context enumerate_context_;
};

PersistentConfig::PersistentConfig(
    Pool& pool, FlashInterface& flash)
    : impl_(&pool, pool, flash) {
}

PersistentConfig::~PersistentConfig() {
}

void PersistentConfig::Command(const gsl::cstring_span& command,
                               const CommandManager::Response& response) {
  Tokenizer tokenizer(command, " ");
  auto cmd = tokenizer.next();
  if (cmd == gsl::ensure_z("enumerate")) {
    impl_->Enumerate(response);
  } else if (cmd == gsl::ensure_z("get")) {
    impl_->Get(tokenizer.remaining(), response);
  } else if (cmd == gsl::ensure_z("set")) {
    impl_->Set(tokenizer.remaining(), response);
  } else if (cmd == gsl::ensure_z("load")) {
    impl_->Load(response);
  } else if (cmd == gsl::ensure_z("write")) {
    impl_->Write(response);
  } else if (cmd == gsl::ensure_z("default")) {
    impl_->Default(response);
  } else {
    impl_->UnknownCommand(cmd, response);
  }
}

void PersistentConfig::Load() {
  impl_->DoLoad();
}

void PersistentConfig::RegisterDetail(
    const gsl::cstring_span& name, SerializableHandlerBase* base,
    StaticFunction<void ()> updated) {
  auto* const element = impl_->elements_.FindOrCreate(
      name, Impl::NamedRegistry::kAllowCreate);
  PoolPtr<Impl::Element> item(&impl_->pool_);
  element->ptr = item.get();
  element->ptr->serializable = base;
  element->ptr->updated = updated;
}

Pool* PersistentConfig::pool() const { return &impl_->pool_; }
