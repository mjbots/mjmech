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

    servo_.AsyncStart(boost::bind(&Selector::HandleStart, this, _1, callback));
  }

  ServoInterface* controller() { return &servo_interface_; }

 private:
  void HandleStart(const boost::system::error_code& ec,
                   bp::object callback) {
    if (ec) {
      callback(false);
    }
    started_ = true;
    callback(true);
  }
  
  boost::asio::io_service service_;
  Factory factory_{service_};
  Servo servo_{service_, factory_};
  HerkuleXServoInterface<Servo> servo_interface_{&servo_};
  bool started_{false};
};

BOOST_PYTHON_MODULE(_legtool) {
  using namespace boost::python;

  class_<ServoInterface::Joint>("ServoInterfaceJoint")
      .def_readwrite("address", &ServoInterface::Joint::address)
      .def_readwrite("angle_deg", &ServoInterface::Joint::angle_deg)
      ;
  
  class_<ServoInterface, boost::noncopyable>("ServoInterface", no_init)
      .def("SetPose", &ServoInterface::SetPose)
      ;

  class_<Selector, boost::noncopyable>("Selector")
      .def("poll", &Selector::poll)
      .def("select_servo", &Selector::select_servo)
      .def("controller", &Selector::controller,
           return_internal_reference<1>())
      ;
}
