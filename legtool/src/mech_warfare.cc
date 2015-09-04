// Copyright 2014-2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "mech_warfare.h"

#include <boost/property_tree/json_parser.hpp>

#include "property_tree_archive.h"

namespace legtool {

RippleConfig MechWarfare::LoadRippleConfig() {
  std::ifstream inf(parameters_->gait_config);
  if (!inf.is_open()) {
    throw SystemError::syserrno(
        "while opening: '" + parameters_->gait_config + "'");
  }

  boost::property_tree::ptree tree;
  boost::property_tree::read_json(inf, tree);
  RippleConfig ripple_config;
  PropertyTreeReadArchive(tree).Accept(&ripple_config);

  // TODO jpieper: Load each leg's IK settings.

  return ripple_config;
}

}
