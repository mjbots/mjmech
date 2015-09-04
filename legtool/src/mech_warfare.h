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

#include "comm_factory.h"
#include "gait_driver.h"
#include "herkulex.h"
#include "herkulex_servo_interface.h"
#include "mjmech_imu_driver.h"
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
  MechWarfare(Context& context)
      : service_(context.service),
        factory_(context.service) {

    servo_base_.reset(new ServoBase(context.service, factory_));
    servo_.reset(new Servo(servo_base_.get()));
    gait_driver_.reset(new GaitDriver(service_,
                                      &context.telemetry_registry,
                                      servo_.get()));
    imu_.reset(new MjmechImuDriver(context));

    parameters_.reset(new Parameters(*servo_base_->parameters(),
                                     *servo_->parameters(),
                                     *gait_driver_->parameters(),
                                     *imu_->parameters()));
  }

  ~MechWarfare() {}

  void AsyncStart(ErrorHandler handler) {
    try {
      RippleConfig ripple_config;
      try {
        ripple_config = LoadRippleConfig();
      } catch (SystemError& se) {
        handler(se.error_code());
        return;
      }

      gait_driver_->SetGait(std::unique_ptr<RippleGait>(
                                new RippleGait(ripple_config)));
    } catch (SystemError& se) {
      handler(se.error_code());
      return;
    }

    // TODO jpieper: Actually start up a network service.

    // TODO jpieper: Make up a class that lets us call our handler on
    // the union of all children starts.

    // TODO jpieper: Make it not so obnoxious to add more children.
    // Currently each child is listed in 6 separate places.

    servo_base_->AsyncStart(handler);
    imu_->AsyncStart([](ErrorCode ec) { FailIf(ec); });
  }

  struct Parameters {
    int port = 13356;

    std::string gait_config;

    ServoBase::Parameters& herkulex;
    Servo::Parameters& servo;
    GaitDriver::Parameters& gait_driver;
    MjmechImuDriver::Parameters& imu;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(LT_NVP(port));
      a->Visit(LT_NVP(gait_config));
      a->Visit(LT_NVP(herkulex));
      a->Visit(LT_NVP(servo));
      a->Visit(LT_NVP(gait_driver));
      a->Visit(LT_NVP(imu));
    }

    Parameters(ServoBase::Parameters& a,
               Servo::Parameters& b,
               GaitDriver::Parameters& c,
               MjmechImuDriver::Parameters& d)
        : herkulex(a),
          servo(b),
          gait_driver(c),
          imu(d) {}
  };

  Parameters* parameters() { return parameters_.get(); }

 private:
  RippleConfig LoadRippleConfig();

  boost::asio::io_service& service_;

  Factory factory_;

  std::unique_ptr<ServoBase> servo_base_;
  std::unique_ptr<Servo> servo_;
  std::unique_ptr<GaitDriver> gait_driver_;
  std::unique_ptr<MjmechImuDriver> imu_;

  std::unique_ptr<Parameters> parameters_;
};
}
