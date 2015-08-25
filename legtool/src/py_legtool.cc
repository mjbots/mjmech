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

#include "comm_factory.h"
#include "gait.h"
#include "herkulex.h"
#include "herkulex_servo_interface.h"
#include "leg_ik.h"
#include "point3d.h"
#include "property_tree_archive.h"
#include "ripple.h"

using namespace legtool;
typedef StreamFactory<StdioGenerator,
                      SerialPortGenerator,
                      TcpClientGenerator> Factory;
typedef HerkuleX<Factory> Servo;
namespace bp = boost::python;

namespace {
bp::object g_runtime_error = bp::eval("RuntimeError");

void HandleCallback(bp::object future,
                    boost::system::error_code ec,
                    bp::object result) {
  if (ec) {
    future.attr("set_exception")(
        g_runtime_error(boost::lexical_cast<std::string>(ec)));
  } else {
    future.attr("set_result")(result);
  }
}

class ServoInterfaceWrapper : boost::noncopyable {
 public:
  ServoInterfaceWrapper(ServoInterface* servo) : servo_(servo) {}
  ServoInterfaceWrapper(const ServoInterfaceWrapper& rhs)
    : servo_(rhs.servo_) {}

  void set_pose(bp::object py_joints,
                bp::object future) {
    bp::list py_addresses = bp::list(py_joints.attr("keys")());
    std::vector<ServoInterface::Joint> joints;
    for (int i = 0; i < bp::len(py_addresses); i++) {
      joints.emplace_back(ServoInterface::Joint{
          bp::extract<int>(py_addresses[i]),
              bp::extract<double>(py_joints[i])});
    }
    servo_->SetPose(joints, [=](boost::system::error_code ec) {
        HandleCallback(future, ec, bp::object());
      });
  }

  void enable_power(bp::object py_addresses,
                    ServoInterface::PowerState power_state,
                    bp::object future) {
    std::vector<int> addresses = GetAddresses(py_addresses);
    servo_->EnablePower(
        power_state, addresses,
        [=](boost::system::error_code ec) {
          HandleCallback(future, ec, bp::object());
        });
  }

  void get_pose(bp::object py_addresses, bp::object future) {
    std::vector<int> addresses = GetAddresses(py_addresses);
    servo_->GetPose(
        addresses,
        [=](boost::system::error_code ec,
            const std::vector<ServoInterface::Joint> joints) {

          bp::dict result;
          for (const auto& joint: joints) {
            result[joint.address] = joint.angle_deg;
          }

          HandleCallback(future, ec, result);
      });
  }

  void get_temperature(bp::object py_addresses, bp::object future) {
    std::vector<int> addresses = GetAddresses(py_addresses);
    servo_->GetTemperature(
        addresses, [=](const boost::system::error_code& ec,
                       const std::vector<ServoInterface::Temperature>& temps) {
          bp::dict result;
          for (const auto& temp: temps) {
            result[temp.address] = temp.temperature_C;
          }

          HandleCallback(future, ec, result);
        });
  }

  void get_voltage(bp::object py_addresses, bp::object future) {
    std::vector<int> addresses = GetAddresses(py_addresses);
    servo_->GetVoltage(
        addresses, [=](const boost::system::error_code& ec,
                       const std::vector<ServoInterface::Voltage>& temps) {
          bp::dict result;
          for (const auto& temp: temps) {
            result[temp.address] = temp.voltage;
          }

          HandleCallback(future, ec, result);
        });
  }

 private:
  static std::vector<int> GetAddresses(bp::object py_addresses) {
    std::vector<int> addresses;
    for (int i = 0; i < bp::len(py_addresses); i++) {
      addresses.push_back(bp::extract<int>(py_addresses[i]));
    }
    return addresses;
  }

  ServoInterface* const servo_;
};

class Selector : boost::noncopyable {
 public:
  void poll() {
    service_.poll();
    service_.reset();
  }

  void select_servo(
      const std::string& servo_type,
      const std::string& serial_port,
      bp::object future) {

    // TODO jpieper: Support gazebo.
    if (servo_type != "herkulex") {
      throw std::runtime_error("we only support herkulex servos for now");
    }

    auto params = servo_.parameters();

    // TODO jpieper: Support TCP with a magic prefix.
    params->stream.type = "serial";
    params->stream.Get<SerialPortGenerator>()->serial_port = serial_port;

    std::cout << "about to async start\n";
    servo_.AsyncStart([=](boost::system::error_code ec) {
        if (ec) {
          future.attr("set_exception")(
              std::runtime_error(boost::lexical_cast<std::string>(ec)));
        } else {
          future.attr("set_result")(bp::object());
        }
      });
  }

  ServoInterfaceWrapper* controller() {
    return &wrapper_;
  }

 private:

  boost::asio::io_service service_;
  Factory factory_{service_};
  Servo servo_{service_, factory_};
  HerkuleXServoInterface<Servo> servo_interface_{&servo_};
  ServoInterfaceWrapper wrapper_{&servo_interface_};
  bool started_{false};
};

template <typename Serializable>
Serializable SerializableReadSettings(bp::object dict) {
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

template <typename Serializable>
void SerializableWriteSettings(const Serializable* object,
                               bp::object out) {
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

boost::shared_ptr<IKSolver> MakeIKSolver(std::auto_ptr<LizardIK> lizard_ik) {
  auto result = boost::shared_ptr<IKSolver>(lizard_ik.get());
  lizard_ik.release();
  return result;
}

LizardIK* MakeLizardIK(const LizardIK::Config& config) {
  return new LizardIK(config);
}
}

BOOST_PYTHON_MODULE(_legtool) {
  using namespace boost::python;

  enum_<ServoInterface::PowerState>("PowerState")
      .value("kPowerFree", ServoInterface::kPowerFree)
      .value("kPowerBrake", ServoInterface::kPowerBrake)
      .value("kPowerEnable", ServoInterface::kPowerEnable)
      ;

  class_<ServoInterface::Joint>("ServoInterfaceJoint")
      .def_readwrite("address", &ServoInterface::Joint::address)
      .def_readwrite("angle_deg", &ServoInterface::Joint::angle_deg)
      ;

  class_<ServoInterfaceWrapper, boost::noncopyable>("ServoInterface", no_init)
      .def("set_pose", &ServoInterfaceWrapper::set_pose)
      .def("enable_power", &ServoInterfaceWrapper::enable_power)
      .def("get_pose", &ServoInterfaceWrapper::get_pose)
      .def("get_temperature", &ServoInterfaceWrapper::get_temperature)
      .def("get_voltage", &ServoInterfaceWrapper::get_voltage)
      ;

  class_<Selector, boost::noncopyable>("Selector")
      .def("poll", &Selector::poll)
      .def("select_servo", &Selector::select_servo)
      .def("controller", &Selector::controller,
           return_internal_reference<1>())
      ;

  class_<Point3D>("Point3D", init<double, double, double>())
      .def(init<>())
      .def_readwrite("x", &Point3D::x)
      .def_readwrite("y", &Point3D::y)
      .def_readwrite("z", &Point3D::z)
      .def("__str_", &Point3D::str)
      .def("length", &Point3D::length)
      .def("length_squared", &Point3D::length_squared)
      .def("scaled", &Point3D::scaled)
      .def("__getitem__", &Point3D::operator[])
      .def(self + self)
      .def(self - self)
      .def(self == self)
      ;

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

  class_<LizardIK, boost::noncopyable>("LizardIK", init<LizardIK::Config>())
      .def("solve", &LizardIK::Solve)
      .def("do_ik", &LizardIK::Solve)
      .def("config", &LizardIK::config,
           return_internal_reference<1>())
      .def("create", &MakeLizardIK,
           return_value_policy<manage_new_object>())
      .staticmethod("create")
      ;

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

  class_<Frame, boost::noncopyable>("Frame")
      .def("map_to_frame", &Frame::MapToFrame<Point3D>)
      .def("map_from_frame", &Frame::MapFromFrame<Point3D>)
      .def("map_to_parent", &Frame::MapToParent<Point3D>)
      .def("map_from_parent", &Frame::MapFromParent<Point3D>)
      ;

  class_<RippleState::Leg>("RippleStateLeg")
      .def_readwrite("point", &RippleState::Leg::point)
      .def_readwrite("mode", &RippleState::Leg::mode)
      .def_readwrite("leg_ik", &RippleState::Leg::leg_ik)
      .def_readwrite("frame", &RippleState::Leg::frame)
      .def_readwrite("swing_start_pos", &RippleState::Leg::swing_start_pos)
      .def_readwrite("swing_end_pos", &RippleState::Leg::swing_end_pos)
      ;

  class_<std::vector<RippleState::Leg> >("RippleStateLegList")
      .def(vector_indexing_suite<std::vector<RippleState::Leg> >())
      ;

  class_<RippleState>("RippleState", init<>())
      .def(init<RippleState>())
      .def_readwrite("phase", &RippleState::phase)
      .def_readwrite("action", &RippleState::action)
      .def_readwrite("legs", &RippleState::legs)
      .def_readwrite("world_frame", &RippleState::world_frame)
      .def_readwrite("robot_frame", &RippleState::robot_frame)
      .def_readwrite("body_frame", &RippleState::body_frame)
      .def_readwrite("cog_frame", &RippleState::cog_frame)
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
      ;

  def("make_iksolver", &MakeIKSolver);
}
