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

#include <tuple>
#include <type_traits>

#include <boost/utility/enable_if.hpp>
#include <boost/property_tree/ptree.hpp>

#include "visitor.h"

namespace legtool {
class PropertyTreeWriteArchive {
 public:
  template <typename Serializable>
  PropertyTreeWriteArchive& Accept(Serializable* object) {
    object->Serialize(this);
    return *this;
  }

  template <typename NameValuePair>
  void Visit(const NameValuePair& pair) {
    VisitHelper(pair, 0);
  }

  template <typename NameValuePair>
  auto VisitHelper(const NameValuePair& pair, int) ->
      decltype(pair.value()->Serialize((PropertyTreeWriteArchive*)nullptr)) {
    PropertyTreeWriteArchive sub;
    sub.Accept(pair.value());
    tree_.put_child(pair.name(), sub.tree());
  }

  template <typename NameValuePair>
  void VisitHelper(const NameValuePair& pair, long) {
    tree_.put(pair.name(), *pair.value());
  }

  boost::property_tree::ptree tree() const { return tree_; }

 private:
  boost::property_tree::ptree tree_;
};
}
