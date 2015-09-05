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

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "gait.h"
#include "py_legtool.h"
#include "ripple.h"

using namespace legtool;
namespace bp = boost::python;

namespace legtool {
bp::object ConvertPtree(const boost::property_tree::ptree& tree) {
  if (tree.empty()) {
    return bp::str(tree.get_value<std::string>());
  } else if (tree.begin()->first.empty()) {
    // This must be a list.
    bp::list result;
    for (auto it = tree.begin(); it != tree.end(); ++it) {
      result.append(ConvertPtree(it->second));
    }
    return result;
  } else {
    bp::dict result;
    for (auto it = tree.begin(); it != tree.end(); ++it) {
      result[it->first] = ConvertPtree(it->second);
    }
    return result;
  }
}
}

namespace {
Frame* GetRippleStateLegFrame(const RippleState::Leg* leg) {
  return leg->frame;
}

const Frame* GetRippleStateWorld(const RippleState* state) {
  return &state->world_frame;
}

const Frame* GetRippleStateRobot(const RippleState* state) {
  return &state->robot_frame;
}

const Frame* GetRippleStateBody(const RippleState* state) {
  return &state->body_frame;
}

const Frame* GetRippleStateCog(const RippleState* state) {
  return &state->cog_frame;
}
}

BOOST_PYTHON_MODULE(_legtool) {
  using namespace boost::python;

  ExportServo();
  ExportTf();
  ExportLegIK();

  enum_<Leg::Mode>("LegMode")
      .value("kStance", Leg::Mode::kStance)
      .value("kSwing", Leg::Mode::kSwing)
      .value("kUnknown", Leg::Mode::kUnknown)
      ;

  class_<Leg::Config>("LegConfig")
      .def_readwrite("mount_mm", &Leg::Config::mount_mm)
      .def_readwrite("idle_mm", &Leg::Config::idle_mm)
      .def_readwrite("leg_ik", &Leg::Config::leg_ik)
      ;

  class_<std::vector<Leg::Config> >("LegConfigList")
      .def(vector_indexing_suite<std::vector<Leg::Config> >())
      ;

  class_<MechanicalConfig>("MechanicalConfig")
      .def_readwrite("leg_config", &MechanicalConfig::leg_config)
      .def_readwrite("body_cog_mm", &MechanicalConfig::body_cog_mm)
      ;

  class_<RippleConfig>("RippleConfig")
      .def(init<RippleConfig>())
      .def_readwrite("mechanical", &RippleConfig::mechanical)
      .def_readwrite("max_cycle_time_s", &RippleConfig::max_cycle_time_s)
      .def_readwrite("lift_height_mm", &RippleConfig::lift_height_mm)
      .def_readwrite("lift_percent", &RippleConfig::lift_percent)
      .def_readwrite("swing_percent", &RippleConfig::swing_percent)
      .def_readwrite("position_margin_percent",
                     &RippleConfig::position_margin_percent)
      .def_readwrite("leg_order", &RippleConfig::leg_order)
      .def_readwrite("body_z_offset_mm", &RippleConfig::body_z_offset_mm)
      .def_readwrite("servo_speed_margin_percent",
                     &RippleConfig::servo_speed_margin_percent)
      .def_readwrite("statically_stable", &RippleConfig::statically_stable)
      .def_readwrite("static_center_factor",
                     &RippleConfig::static_center_factor)
      .def_readwrite("static_stable_factor",
                     &RippleConfig::static_stable_factor)
      .def_readwrite("static_margin_mm", &RippleConfig::static_margin_mm)
      .def_readwrite("servo_speed_dps", &RippleConfig::servo_speed_dps)
      .def("read_settings", &SerializableReadSettings<RippleConfig>)
      .staticmethod("read_settings")
      .def("write_settings", &SerializableWriteSettings<RippleConfig>)
      ;

  class_<JointCommand::Joint>("JointCommandJoint")
      .def_readwrite("servo_number", &JointCommand::Joint::servo_number)
      .def_readwrite("angle_deg", &JointCommand::Joint::angle_deg)
      ;

  class_<JointCommand>("JointCommand")
      .def_readwrite("joints", &JointCommand::joints)
      ;

  class_<std::vector<JointCommand::Joint> >("JointCommandJointList")
      .def(vector_indexing_suite<std::vector<JointCommand::Joint> >())
      ;

  enum_<RippleGait::Result>("RippleGaitResult")
      .value("kValid", RippleGait::Result::kValid)
      .value("kNotSupported", RippleGait::Result::kNotSupported)
      ;

  class_<Command>("Command")
      .def(init<Command>())
      .def_readwrite("translate_x_mm_s", &Command::translate_x_mm_s)
      .def_readwrite("translate_y_mm_s", &Command::translate_y_mm_s)
      .def_readwrite("rotate_deg_s", &Command::rotate_deg_s)
      .def_readwrite("body_x_mm", &Command::body_x_mm)
      .def_readwrite("body_y_mm", &Command::body_y_mm)
      .def_readwrite("body_z_mm", &Command::body_z_mm)
      .def_readwrite("body_pitch_deg", &Command::body_pitch_deg)
      .def_readwrite("body_roll_deg", &Command::body_roll_deg)
      .def_readwrite("body_yaw_deg", &Command::body_yaw_deg)
      .def_readwrite("lift_height_percent", &Command::lift_height_percent)
      ;

  class_<RippleState::Leg>("RippleStateLeg")
      .def_readwrite("point", &RippleState::Leg::point)
      .def_readwrite("mode", &RippleState::Leg::mode)
      .add_property("leg_ik", &RippleState::Leg::leg_ik)
      .def_readwrite("swing_start_pos", &RippleState::Leg::swing_start_pos)
      .def_readwrite("swing_end_pos", &RippleState::Leg::swing_end_pos)
      .add_property("frame",
                    make_function(&GetRippleStateLegFrame,
                                  return_internal_reference<1>()))
      ;

  class_<std::vector<RippleState::Leg> >("RippleStateLegList")
      .def(vector_indexing_suite<std::vector<RippleState::Leg> >())
      ;

  class_<RippleState>("RippleState", init<>())
      .def(init<RippleState>())
      .def_readwrite("phase", &RippleState::phase)
      .def_readwrite("action", &RippleState::action)
      .def_readwrite("legs", &RippleState::legs)
      .def("make_shoulder", &RippleState::MakeShoulder)
      .add_property("world_frame",
                    make_function(&GetRippleStateWorld,
                                  return_internal_reference<1>()))
      .add_property("robot_frame",
                    make_function(&GetRippleStateRobot,
                                  return_internal_reference<1>()))
      .add_property("body_frame",
                    make_function(&GetRippleStateBody,
                                  return_internal_reference<1>()))
      .add_property("cog_frame",
                    make_function(&GetRippleStateCog,
                                  return_internal_reference<1>()))
      ;

  class_<Options>("Options")
      .def_readwrite("cycle_time_s", &Options::cycle_time_s)
      .def_readwrite("servo_speed_dps", &Options::servo_speed_dps)
      ;

  class_<RippleGait, boost::noncopyable>("RippleGait", init<RippleConfig>())
      .def("advance_phase", &RippleGait::AdvancePhase)
      .def("advance_time", &RippleGait::AdvanceTime)
      .def("set_command", &RippleGait::SetCommand)
      .def("get_idle_state", &RippleGait::GetIdleState)
      .def("state", &RippleGait::state,
           return_internal_reference<1>())
      .def("set_state", &RippleGait::SetState)
      .def("options", &RippleGait::options,
           return_internal_reference<1>())
      .def("command", &RippleGait::command,
           return_internal_reference<1>())
      .def("make_joint_command", &RippleGait::MakeJointCommand)
      ;
}
