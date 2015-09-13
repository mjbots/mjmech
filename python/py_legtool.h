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

#include <boost/property_tree/json_parser.hpp>
#include <boost/python.hpp>

#include "base/property_tree_archive.h"

void ExportServo();
void ExportTf();
void ExportLegIK();

namespace legtool {
boost::python::object ConvertPtree(const boost::property_tree::ptree&);

template <typename Serializable>
Serializable SerializableReadSettings(boost::python::object dict) {
  namespace bp = boost::python;

  Serializable object{};
  bp::object json = bp::import("json");
  bp::object json_data = json.attr("dumps")(dict);
  std::string json_str = bp::extract<std::string>(bp::str(json_data));

  boost::property_tree::ptree tree;
  std::istringstream istr(json_str);
  boost::property_tree::read_json(istr, tree);

  PropertyTreeReadArchive(tree).Accept(&object);
  return object;
}

template <typename Serializable>
void SerializableWriteSettings(const Serializable* object,
                               boost::python::object out) {
  namespace bp = boost::python;

  auto tree = PropertyTreeWriteArchive().
      Accept(const_cast<Serializable*>(object)).tree();
  // Copy this property tree into the python output dict.
  bp::object result = ConvertPtree(tree);
  bp::dict result_dict(result);
  bp::list keys = result_dict.keys();
  for (int i = 0; i < bp::len(keys); ++i) {
    out[keys[i]] = result_dict[keys[i]];
  }
}

}
