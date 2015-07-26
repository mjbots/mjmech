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

#include "leg_ik.h"

#include <boost/test/auto_unit_test.hpp>

namespace {
using namespace legtool;
LizardIK::Config MakeConfig() {
  LizardIK::Config r;

  r.coxa.length_mm = 50;
  r.femur.length_mm = 40;
  r.tibia_length_mm = 30;
  r.coxa.min_deg = -90;
  r.coxa_idle_deg = 0;
  r.coxa_max_deg = 0;
  r.femur_min_deg = -90;
  r.femur_idle_deg = 0;
  r.femur_max_deg = 90;
  r.tibia_min_deg = -90;
  r.tibia_idle_deg = 0;
  r.tibia_max_deg = 90;

  return r;
}
}

BOOST_AUTO_TEST_CASE(TestLizard3Dof) {
}
