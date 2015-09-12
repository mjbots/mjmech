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

#include <boost/property_tree/json_parser.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "leg_ik.h"
#include "property_tree_archive.h"
#include "py_legtool.h"

using namespace legtool;

void ExportLegIK() {
  using namespace boost::python;

  class_<JointAngles::Joint>("JointAnglesJoint")
      .def_readwrite("ident", &JointAngles::Joint::ident)
      .def_readwrite("angle_deg", &JointAngles::Joint::angle_deg)
      ;

  class_<std::vector<JointAngles::Joint> >("JointAnglesList")
      .def(vector_indexing_suite<std::vector<JointAngles::Joint> >())
      ;

  class_<std::vector<int> >("vector_int")
      .def(vector_indexing_suite<std::vector<int> >())
      ;

  class_<std::vector<std::vector<int > > >("vector_vector_int")
      .def(vector_indexing_suite<std::vector<std::vector<int> > >())
      ;

  class_<JointAngles>("JointAngles")
      .def_readwrite("joints", &JointAngles::joints)
      .def("valid", &JointAngles::Valid)
      .def("largest_change_deg", &JointAngles::GetLargestChangeDeg)
      ;

  class_<IKSolver,
         boost::shared_ptr<IKSolver>,
         boost::noncopyable>("IKSolver", no_init)
      .def("solve", &IKSolver::Solve)
      .def("do_ik", &IKSolver::Solve)
      ;

  class_<LizardIK::Config::Joint>("LizardIKConfigJoint")
      .def_readwrite("min_deg", &LizardIK::Config::Joint::min_deg)
      .def_readwrite("idle_deg", &LizardIK::Config::Joint::idle_deg)
      .def_readwrite("max_deg", &LizardIK::Config::Joint::max_deg)
      .def_readwrite("length_mm", &LizardIK::Config::Joint::length_mm)
      .def_readwrite("sign", &LizardIK::Config::Joint::sign)
      .def_readwrite("ident", &LizardIK::Config::Joint::ident)
      ;

  class_<LizardIK::Config>("LizardIKConfig")
      .def_readwrite("coxa", &LizardIK::Config::coxa)
      .def_readwrite("femur", &LizardIK::Config::femur)
      .def_readwrite("tibia", &LizardIK::Config::tibia)
      .def_readwrite("servo_speed_dps", &LizardIK::Config::servo_speed_dps)
      .def("write_settings", &SerializableWriteSettings<LizardIK::Config>)
      ;

  class_<LizardIK,
         boost::shared_ptr<LizardIK>,
         bases<IKSolver>,
         boost::noncopyable>("LizardIK", init<LizardIK::Config>())
      .def("solve", &LizardIK::Solve)
      .def("do_ik", &LizardIK::Solve)
      .def("config", &LizardIK::config,
           return_internal_reference<1>())
      ;

  class_<MammalIK::Config::Joint>("MammalIKConfigJoint")
      .def_readwrite("min_deg", &MammalIK::Config::Joint::min_deg)
      .def_readwrite("idle_deg", &MammalIK::Config::Joint::idle_deg)
      .def_readwrite("max_deg", &MammalIK::Config::Joint::max_deg)
      .def_readwrite("sign", &MammalIK::Config::Joint::sign)
      .def_readwrite("length_mm", &MammalIK::Config::Joint::length_mm)
      .def_readwrite("ident", &MammalIK::Config::Joint::ident)
      ;

  class_<MammalIK::Config>("MammalIKConfig")
      .def_readwrite("femur_attachment_mm",
                     &MammalIK::Config::femur_attachment_mm)
      .def_readwrite("shoulder", &MammalIK::Config::shoulder)
      .def_readwrite("femur", &MammalIK::Config::femur)
      .def_readwrite("tibia", &MammalIK::Config::tibia)
      .def_readwrite("invert", &MammalIK::Config::invert)
      .def_readwrite("servo_speed_dps", &MammalIK::Config::servo_speed_dps)
      .def("write_settings", &SerializableWriteSettings<MammalIK::Config>)
      ;

  class_<MammalIK::ForwardResult>("MammalIKForwardResult")
      .def_readonly("shoulder", &MammalIK::ForwardResult::shoulder)
      .def_readonly("femur", &MammalIK::ForwardResult::femur)
      .def_readonly("tibia", &MammalIK::ForwardResult::tibia)
      .def_readonly("end", &MammalIK::ForwardResult::end)
      ;

  class_<MammalIK,
         boost::shared_ptr<MammalIK>,
         bases<IKSolver>,
         boost::noncopyable>("MammalIK", init<MammalIK::Config>())
      .def("solve", &MammalIK::Solve)
      .def("do_ik", &MammalIK::Solve)
      .def("forward", &MammalIK::Forward)
      .def("config", &MammalIK::config,
           return_internal_reference<1>())
         ;
}
