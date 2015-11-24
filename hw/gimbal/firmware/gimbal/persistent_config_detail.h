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

#include <cstdio>
#include <type_traits>

#include "base/gsl/gsl-lite.h"
#include "base/visit_archive.h"

#include "async_stream.h"
#include "tokenizer.h"

namespace detail {
struct EnumerateArchive : public mjmech::base::VisitArchive<EnumerateArchive> {
  struct Context {
    gsl::cstring_span root_prefix;
    gsl::string_span buffer;
    uint16_t current_field_index_to_write = 0;
    AsyncWriteStream* stream = nullptr;
    ErrorCallback callback;
    StaticFunction<bool ()> evaluate_enumerate_archive;
  };

  EnumerateArchive(Context* context,
                   gsl::cstring_span prefix,
                   uint16_t* current_index,
                   bool* done,
                   EnumerateArchive* parent)
      : context_(context),
        prefix_(prefix),
        parent_(parent),
        current_index_(current_index),
        done_(done) {}

  template <typename NameValuePair>
  void Visit(const NameValuePair& pair) {
    if (*done_) { return; }
    VisitArchive<EnumerateArchive>::Visit(pair);
  }

  template <typename NameValuePair>
  void VisitScalar(const NameValuePair& pair) {
    auto old_index = *current_index_;
    (*current_index_)++;

    if (old_index == context_->current_field_index_to_write) {
      *done_ = true;

      gsl::cstring_span data = FormatField(
          context_->buffer, gsl::ensure_z(pair.name()), *pair.value());
      AsyncWrite(*context_->stream, data, [ctx = this->context_](int error) {
          if (error) { ctx->callback(error); return; }
          ctx->current_field_index_to_write++;

          bool done = ctx->evaluate_enumerate_archive();

          if (!done) {
            // We have finished with everything.
            ctx->callback(0);
          } else {
            // This archive call should have enqueued another
            // callback, we are done.
          }
        });
    }
  }

  template <typename NameValuePair>
  void VisitSerializable(const NameValuePair& pair) {
    EnumerateArchive(
        context_, gsl::ensure_z(pair.name()), current_index_, done_, this).
        Accept(pair.value());
  }

  template <typename Iterator>
  int FormatPrefix(Iterator* current, Iterator* end, EnumerateArchive* ea) {
    if (ea->parent_) { FormatPrefix(current, end, ea->parent_); }

    // Would we overflow?
    if ((ea->prefix_.size() + 2) > std::distance(*current, *end)) { return 1; }

    for (auto it = ea->prefix_.begin(); it != ea->prefix_.end(); ++it) {
      **current = *it;
      ++(*current);
    }

    **current = '.';
    ++(*current);
    return 0;
  }

  template <typename T>
  gsl::cstring_span FormatField(gsl::string_span buffer,
                                gsl::cstring_span name,
                                T value) {
    auto it = buffer.begin();
    auto end = buffer.end();

    if (FormatPrefix(&it, &end, this)) {
      return gsl::cstring_span();
    }
    if ((name.size() + 2) > std::distance(it, end)) {
      return gsl::cstring_span();
    }

    for (auto nit = name.begin(); nit != name.end(); ++nit) {
      *it = *nit;
      ++it;
    }

    *it = ' ';
    ++it;

    FormatValue(&it, &end, value);
    *it = '\n';
    ++it;
    return gsl::cstring_span(buffer.begin(), it);
  }

  template <typename Iterator, typename T>
  void FormatValue(Iterator* current, Iterator* end, T value) {
    int result = snprintf(&(**current), std::distance(*current, *end),
                          GetFormatSpecifier(value), value);
    (*current) += result;
  }

  template <typename T>
  const char* GetFormatSpecifier(T) const { return "%d"; }

  const char* GetFormatSpecifier(float) const { return "%f"; }

 private:
  Context* const context_;
  const gsl::cstring_span prefix_;
  EnumerateArchive* const parent_;
  uint16_t* const current_index_;
  bool* const done_;
};

template <typename Derived>
struct ItemArchive : public mjmech::base::VisitArchive<Derived> {
  ItemArchive(const gsl::cstring_span& key) {
    Tokenizer tokenizer(key, ".");
    my_key_ = tokenizer.next();
    remaining_key_ = tokenizer.remaining();
  }

  template <typename NameValuePair>
  void Visit(const NameValuePair& pair) {
    if (found_) { return; }
    if (my_key_ != gsl::ensure_z(pair.name())) { return; }
    found_ = true;

    mjmech::base::VisitArchive<Derived>::Visit(pair);
  }

  gsl::cstring_span my_key_;
  gsl::cstring_span remaining_key_;
  bool found_ = false;
};

struct SetArchive : public ItemArchive<SetArchive> {
  SetArchive(const gsl::cstring_span& key,
             const gsl::cstring_span& value)
      : ItemArchive(key), value_(value) {}

  template <typename NameValuePair>
  void VisitScalar(const NameValuePair& pair) {
    using Tref = decltype(*pair.value());
    using T = typename std::remove_reference<Tref>::type;
    pair.set_value(ParseValue<T>(value_));
  }

  template <typename NameValuePair>
  void VisitSerializable(const NameValuePair& pair) {
    SetArchive(remaining_key_, value_).Accept(pair.value());
  }

 private:
  template <typename T>
  T ParseValue(const gsl::cstring_span& value) const {
    return std::strtol(&*value.begin(), nullptr, 0);
  }

  const gsl::cstring_span value_;
};

template <>
float SetArchive::ParseValue<float>(const gsl::cstring_span& value) const {
  return std::strtof(&*value.begin(), nullptr);
}

template <>
double SetArchive::ParseValue<double>(const gsl::cstring_span& value) const {
  return std::strtod(&*value.begin(), nullptr);
}

struct ReadArchive : public ItemArchive<ReadArchive> {
  ReadArchive(const gsl::cstring_span& key,
              const gsl::string_span& buffer,
              AsyncWriteStream& stream,
              ErrorCallback callback)
      : ItemArchive(key),
        buffer_(buffer),
        stream_(stream),
        callback_(callback) {
  }

  template <typename NameValuePair>
  void VisitScalar(const NameValuePair& pair) {
    auto out_buffer = EmitValue(*pair.value());

    AsyncWrite(stream_, out_buffer, callback_);
  }

  template <typename NameValuePair>
  void VisitSerializable(const NameValuePair& pair) {
    ReadArchive(remaining_key_, buffer_, stream_, callback_).
        Accept(pair.value());
  }

  template <typename T>
  gsl::cstring_span EmitValue(T value) {
    int out_size = std::snprintf(
        &*buffer_.begin(), buffer_.size(), "%d", value);
    return gsl::cstring_span(buffer_.begin(), buffer_.begin() + out_size);
  }

  const gsl::string_span buffer_;
  AsyncWriteStream& stream_;
  ErrorCallback callback_;
};

template <>
gsl::cstring_span ReadArchive::EmitValue<float>(float value) {
  int out_size = std::snprintf(
      &*buffer_.begin(), buffer_.size(), "%f", value);
  return gsl::cstring_span(buffer_.begin(), buffer_.begin() + out_size);
}
}
