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
/// A pstring is a 16 byte unsigned integer followed by that many
/// bytes of data.


class PersistentConfig::Impl {
 public:
  Impl(Pool* pool) : pool_(pool) {}

  enum {
    kMaxGroups = 8,
  };

  struct Element {
    gsl::cstring_span name;
    Base* base = nullptr;
  };

  enum CreateMode {
    kAllowCreate,
    kFindOnly,
  };

  Element* FindOrCreate(const gsl::cstring_span& name, CreateMode create_mode) {
    for (auto& element: elements_) {
      if (element.name.size() == 0) {
        switch (create_mode) {
          case kAllowCreate: {
            element.name = name;
            return &element;
          }
          case kFindOnly: {
            return nullptr;
          }
        }
      } else if (element.name == name) {
        return &element;
      }
    }
    // Whoops, we ran out of space.
    Expects(false);
    return nullptr;
  }

  void Enumerate(AsyncWriteStream& stream, ErrorCallback callback) {
    current_write_stream_ = &stream;
    current_callback_ = callback;
    current_enumerate_index_ = 0;

    EnumerateCallback(0);
  }

  void EnumerateCallback(int error) {
    if (error) { current_callback_(error); return; }

    if (current_enumerate_index_ >= elements_.size()) {
      WriteOK(*current_write_stream_, current_callback_);
      return;
    }

    Element* const element = &elements_[current_enumerate_index_];
    if (element->name.size() == 0) {
      WriteOK(*current_write_stream_, current_callback_);
      return;
    }

    current_enumerate_index_++;

    element->base->Enumerate(&this->enumerate_context_,
                             this->send_buffer_,
                             element->name,
                             *this->current_write_stream_,
                             [this](int err) { this->EnumerateCallback(err); });
  }

  void Get(AsyncWriteStream& stream, const gsl::cstring_span& field,
           ErrorCallback callback) {
    Tokenizer tokenizer(field, ".");
    auto group = tokenizer.next();
    Element* const element = FindOrCreate(group, kFindOnly);
    if (element == nullptr) {
      WriteMessage(stream, gsl::ensure_z("unknown group\n"), callback);
    } else {
      current_write_stream_ = &stream;
      current_callback_ = callback;
      int err =
          element->base->Read(
              send_buffer_, tokenizer.remaining(), stream,
              [this](int error) {
                if (error) { this->current_callback_(error); return; }
                WriteMessage(*this->current_write_stream_,
                             gsl::ensure_z("\n"),
                             this->current_callback_);
              });
      if (err) {
        WriteMessage(stream, gsl::ensure_z("error reading\n"), callback);
      }
    }
  }

  void Set(AsyncWriteStream& stream, const gsl::cstring_span& command,
           ErrorCallback callback) {
    Tokenizer tokenizer(command, ".");
    auto group = tokenizer.next();
    Element* const element = FindOrCreate(group, kFindOnly);
    if (element == nullptr) {
      WriteMessage(stream, gsl::ensure_z("unknown group\n"), callback);
    } else {
      Tokenizer name_value(tokenizer.remaining(), " ");
      auto key = name_value.next();
      auto value = name_value.remaining();
      int result = element->base->Set(key, value);
      if (result == 0) {
        WriteOK(stream, callback);
      } else {
        WriteMessage(stream, gsl::ensure_z("error setting\n"), callback);
      }
    }
  }

  void Load(AsyncWriteStream& stream, ErrorCallback callback) {
    WriteOK(stream, callback);
  }

  void Write(AsyncWriteStream& stream, ErrorCallback callback) {
    WriteOK(stream, callback);
  }

  void WriteOK(AsyncWriteStream& stream,
               ErrorCallback callback) {
    WriteMessage(stream, gsl::ensure_z("OK\n"), callback);
  }

  void WriteMessage(AsyncWriteStream& stream,
                    const gsl::cstring_span& message,
                    ErrorCallback callback) {
    AsyncWrite(stream, message, callback);
  }

  void UnknownCommand(AsyncWriteStream& stream,
                      const gsl::cstring_span& command,
                      ErrorCallback callback) {
    AsyncWrite(stream, gsl::ensure_z("unknown command\n"), callback);
  }

  Pool* const pool_;
  std::array<Element, kMaxGroups> elements_;

  // TODO jpieper: This buffer could be shared with other things that
  // have the same output stream, as only one should be writing at a
  // time anyways.
  char send_buffer_[256] = {};

  AsyncWriteStream* current_write_stream_ = nullptr;
  ErrorCallback current_callback_;
  std::size_t current_enumerate_index_ = 0;
  detail::EnumerateArchive::Context enumerate_context_;
};

PersistentConfig::PersistentConfig(Pool* pool) : impl_(pool, pool) {
}

PersistentConfig::~PersistentConfig() {
}

void PersistentConfig::Command(const gsl::cstring_span& command,
                               AsyncWriteStream& stream,
                               ErrorCallback callback) {
  Tokenizer tokenizer(command, " ");
  auto cmd = tokenizer.next();
  if (cmd == gsl::ensure_z("enumerate")) {
    impl_->Enumerate(stream, callback);
  } else if (cmd == gsl::ensure_z("get")) {
    impl_->Get(stream, tokenizer.remaining(), callback);
  } else if (cmd == gsl::ensure_z("set")) {
    impl_->Set(stream, tokenizer.remaining(), callback);
  } else if (cmd == gsl::ensure_z("load")) {
    impl_->Load(stream, callback);
  } else if (cmd == gsl::ensure_z("write")) {
    impl_->Write(stream, callback);
  } else {
    impl_->UnknownCommand(stream, cmd, callback);
  }
}

void PersistentConfig::Load() {
}

void PersistentConfig::RegisterDetail(
    const gsl::cstring_span& name, Base* base) {
  auto* element = impl_->FindOrCreate(name, Impl::kAllowCreate);
  element->base = base;
}

Pool* PersistentConfig::pool() const { return impl_->pool_; }
