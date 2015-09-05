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

#include <boost/asio/ip/udp.hpp>
#include <boost/property_tree/ptree.hpp>

#include "comm_factory.h"
#include "gait_driver.h"
#include "handler_util.h"
#include "herkulex.h"
#include "herkulex_servo_interface.h"
#include "mjmech_imu_driver.h"
#include "parameters_archive.h"
#include "ripple.h"

namespace legtool {
/// Accepts json formatted commands over the network and uses that to
/// sequence gaits and firing actions.
///
/// NOTE jpieper: This could also manage video if we had a way of
/// managing it from C++.
class MechWarfare : boost::noncopyable {
 public:
  typedef StreamFactory<StdioGenerator,
                        SerialPortGenerator,
                        TcpClientGenerator,
                        PipeGenerator> Factory;
  typedef HerkuleX<Factory> ServoBase;
  typedef HerkuleXServoInterface<ServoBase> Servo;

  template <typename Context>
  MechWarfare(Context& context) : MechWarfare(context.service) {
    m_.imu.reset(new MjmechImuDriver(context));
    m_.gait_driver.reset(new GaitDriver(service_,
                                        &context.telemetry_registry,
                                        m_.servo.get()));
  }

  MechWarfare(boost::asio::io_service&);
  ~MechWarfare() {}

  void AsyncStart(ErrorHandler handler);

  struct Parameters {
    int port = 13356;
    std::map<std::string, bool> enabled;

    std::string gait_config;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(LT_NVP(port));
      a->Visit(LT_NVP(gait_config));
      for (auto& pair: enabled) {
        a->Visit(MakeNameValuePair(
                     &pair.second, (pair.first + "_enable").c_str()));
      }
      ParametersArchive<Archive>(a).Accept(&parent->m_);
    }

    Parameters(MechWarfare* mw) : parent(mw) {}

    MechWarfare* const parent;
  };

  Parameters* parameters() { return &parameters_; }

 private:
  RippleConfig LoadRippleConfig();
  void NetworkListen();
  void StartRead();
  void HandleRead(ErrorCode, std::size_t);
  void HandleMessage(const boost::property_tree::ptree&);
  void HandleMessageGait(const boost::property_tree::ptree&);

  boost::asio::io_service& service_;

  Factory factory_;

  struct Members {
    std::unique_ptr<ServoBase> servo_base;
    std::unique_ptr<Servo> servo;
    std::unique_ptr<GaitDriver> gait_driver;
    std::unique_ptr<MjmechImuDriver> imu;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(LT_NVP(servo_base));
      a->Visit(LT_NVP(servo));
      a->Visit(LT_NVP(gait_driver));
      a->Visit(LT_NVP(imu));
    }
  } m_;

  Parameters parameters_{this};
  boost::asio::ip::udp::socket server_;
  char receive_buffer_[3000] = {};
  boost::asio::ip::udp::endpoint receive_endpoint_;
};
}
