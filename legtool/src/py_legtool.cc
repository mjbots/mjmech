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

using namespace legtool;
typedef StreamFactory<StdioGenerator,
                      SerialPortGenerator,
                      TcpClientGenerator> Factory;
typedef HerkuleX<Factory> Servo;
namespace bp = boost::python;

class ServoInterfaceWrapper : boost::noncopyable {
 public:
  ServoInterfaceWrapper(ServoInterface* servo) : servo_(servo) {}
  ServoInterfaceWrapper(const ServoInterfaceWrapper& rhs)
    : servo_(rhs.servo_) {}

  void SetPose(const std::vector<ServoInterface::Joint>& joints,
               bp::object callback) {
    servo_->SetPose(joints, [=](boost::system::error_code ec) {
        if (ec) { throw boost::system::system_error(ec); }
        callback();
      });
  }

  void EnablePower(ServoInterface::PowerState power_state,
                   bp::object py_addresses,
                   bp::object callback) {
    std::vector<int> addresses = GetAddresses(py_addresses);
    servo_->EnablePower(
        power_state, addresses,
        [=](boost::system::error_code ec) {
          if (ec) { throw boost::system::system_error(ec); }
          callback();
        });
  }

  void GetPose(bp::object py_addresses, bp::object callback) {
    std::vector<int> addresses = GetAddresses(py_addresses);
    servo_->GetPose(addresses, [=](
                        boost::system::error_code ec,
                        const std::vector<ServoInterface::Joint> joints) {
        if (ec) { throw boost::system::system_error(ec); }
        bp::dict result;
        for (const auto& joint: joints) {
          result[joint.address] = joint.angle_deg;
        }

        callback(result);
      });
  }

  void GetTemperature(bp::object py_addresses, bp::object callback) {
    std::vector<int> addresses = GetAddresses(py_addresses);
    servo_->GetTemperature(
        addresses, [=](const boost::system::error_code& ec,
                       const std::vector<ServoInterface::Temperature>& temps) {
          if (ec) { throw boost::system::system_error(ec); }
          bp::dict result;
          for (const auto& temp: temps) {
            result[temp.address] = temp.temperature_C;
          }

          callback(result);
        });
  }

  void GetVoltage(bp::object py_addresses, bp::object callback) {
    std::vector<int> addresses = GetAddresses(py_addresses);
    servo_->GetVoltage(
        addresses, [=](const boost::system::error_code& ec,
                       const std::vector<ServoInterface::Voltage>& temps) {
          if (ec) { throw boost::system::system_error(ec); }
          bp::dict result;
          for (const auto& temp: temps) {
            result[temp.address] = temp.voltage;
          }

          callback(result);
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
      bp::object callback) {

    // TODO jpieper: Support gazebo.
    if (servo_type != "herkulex") {
      throw std::runtime_error("we only support herkulex servos for now");
    }

    auto params = servo_.parameters();

    // TODO jpieper: Support TCP with a magic prefix.
    params->stream.type = "serial";
    params->stream.Get<SerialPortGenerator>()->serial_port = serial_port;

    servo_.AsyncStart([=](boost::system::error_code ec) {
        if (ec) { throw boost::system::system_error(ec); }
        callback();
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
      .def("SetPose", &ServoInterfaceWrapper::SetPose)
      .def("EnablePower", &ServoInterfaceWrapper::EnablePower)
      .def("GetPose", &ServoInterfaceWrapper::GetPose)
      .def("GetTemperature", &ServoInterfaceWrapper::GetTemperature)
      .def("GetVoltage", &ServoInterfaceWrapper::GetVoltage)
      ;

  class_<Selector, boost::noncopyable>("Selector")
      .def("poll", &Selector::poll)
      .def("select_servo", &Selector::select_servo)
      .def("controller", &Selector::controller,
           return_internal_reference<1>())
      ;
}
