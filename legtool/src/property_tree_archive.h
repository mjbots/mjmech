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

#include <boost/property_tree/ptree.hpp>

#include "visitor.h"

namespace legtool {
class PropertyTreeWriteArchive {
 public:
  typedef boost::property_tree::ptree ptree;

  template <typename Serializable>
  PropertyTreeWriteArchive& Accept(Serializable* object) {
    object->Serialize(this);
    return *this;
  }

  template <typename NameValuePair>
  void Visit(const NameValuePair& pair) {
    ptree tree;
    VisitHelper(pair.value(), &tree, 0);
    tree_.put_child(pair.name(), tree);
  }

  template <typename Value>
  auto VisitHelper(Value* value, ptree* tree, int) ->
      decltype(value->Serialize((PropertyTreeWriteArchive*)nullptr)) {
    PropertyTreeWriteArchive sub;
    sub.Accept(value);
    *tree = sub.tree();
  }

  template <typename Value>
  void VisitHelper(std::vector<Value>* value_vector,
                   ptree* tree, int) {
    for (int i = 0; i < value_vector->size(); i++) {
      ptree element;
      VisitHelper(&(*value_vector)[i], &element, 0);
      tree->push_back(ptree::value_type("", element));
    }
  }

  template <typename Value>
  void VisitHelper(Value* value, ptree* tree, long) {
    tree->put_value(*value);
  }

  boost::property_tree::ptree tree() const { return tree_; }

 private:
  boost::property_tree::ptree tree_;
};
}
