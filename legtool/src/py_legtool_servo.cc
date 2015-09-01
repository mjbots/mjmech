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

#include <boost/python.hpp>

#include "comm_factory.h"
#include "herkulex.h"
#include "herkulex_servo_interface.h"
#include "py_legtool.h"

using namespace legtool;
namespace bp = boost::python;

typedef StreamFactory<StdioGenerator,
                      SerialPortGenerator,
                      TcpClientGenerator> Factory;
typedef HerkuleX<Factory> Servo;

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
      int address = bp::extract<int>(py_addresses[i]);
      joints.emplace_back(ServoInterface::Joint{
          address,
              bp::extract<double>(py_joints[address])});
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
}

void ExportServo() {
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
}
