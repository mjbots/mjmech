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

#include "visitor.h"

namespace legtool {
class ProgramOptionsArchive {
 public:
  ProgramOptionsArchive(boost::program_options::options_description* description,
                        std::string prefix = "")
      : description_(description),
        prefix_(prefix) {}

  template <typename Serializable>
  void Accept(Serializable* serializable) {
    serializable->Serialize(this);
  }

  template <typename NameValuePair>
  void Visit(const NameValuePair& pair) {
    VisitHelper(pair.value(), pair.name(), 0);
  }

  template <typename Value>
  auto VisitHelper(Value* value, const char* name, int) ->
      decltype(value->Serialize((ProgramOptionsArchive*)nullptr)) {
    ProgramOptionsArchive sub(description_, prefix_ + name + ".");
    sub.Accept(value);
  }

  template <typename Value>
  void VisitHelper(Value* value, const char* name, long) {
    (*description_).add_options()((prefix_ + name).c_str(),
                                  boost::program_options::value(value));
  }

  boost::program_options::options_description* const description_;
  const std::string prefix_;
};
}
