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

#include <cstring>

#include "base/gsl/gsl-lite.h"

/// A simple gsl::string_span based tokenizer.  It can split on
/// multiple delimiters, and reports multiple consecutive delimiters
/// as empty tokens.
class Tokenizer {
 public:
  Tokenizer(const gsl::cstring_span& source, const char* delimiters)
      : source_(source),
        delimiters_(delimiters),
        position_(source_.cbegin()) {}

  gsl::cstring_span next() {
    if (position_ == source_.end()) { return gsl::cstring_span(); }
    const auto start = position_;
    auto next = position_;
    bool found = false;
    for (; next != source_.end(); ++next) {
      if (std::strchr(delimiters_, *next) != nullptr) {
        position_ = next;
        ++position_;
        found = true;
        break;
      }
    }
    if (!found) { position_ = next; }
    return gsl::cstring_span(start, next);
  }

  gsl::cstring_span remaining() const {
    return gsl::cstring_span(position_, source_.end());
  }

 private:
  const gsl::cstring_span source_;
  const char* const delimiters_;
  gsl::cstring_span::const_iterator position_;
};
