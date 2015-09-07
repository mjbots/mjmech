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

#pragma once

#include <boost/asio/io_service.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/signals2/signal.hpp>

#include "comm.h"
#include "point3d.h"
#include "visitor.h"

namespace legtool {
/// Driver for the MAX21000 MMA8451Q driver on the mjmech
/// daughterboards.
class MjmechImuDriver : boost::noncopyable {
 public:
  template <typename Context>
  MjmechImuDriver(Context& context)
      : MjmechImuDriver(context.service,
                        &context.telemetry_registry) {}

  template <typename TelemetryRegistry>
  MjmechImuDriver(boost::asio::io_service& service,
                  TelemetryRegistry* telemetry_registry)
      : MjmechImuDriver(service) {
    telemetry_registry->Register("imu", &imu_data_signal_);
  }

  MjmechImuDriver(boost::asio::io_service&);
  ~MjmechImuDriver();

  void AsyncStart(ErrorHandler handler);

  struct Parameters {
    std::string i2c_device;
    int gyro_address = 0x59;
    int accel_address = 0x1d;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(LT_NVP(i2c_device));
      a->Visit(LT_NVP(gyro_address));
      a->Visit(LT_NVP(accel_address));
    }
  };

  Parameters* parameters() { return &parameters_; }

  struct ImuData {
    boost::posix_time::ptime timestamp;
    Point3D accel_mps2;
    Point3D body_rate_deg_s;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(LT_NVP(timestamp));
      a->Visit(LT_NVP(accel_mps2));
      a->Visit(LT_NVP(body_rate_deg_s));
    }
  };

 private:
  boost::signals2::signal<void (const ImuData*)> imu_data_signal_;
  Parameters parameters_;

  class Impl;
  std::unique_ptr<Impl> impl_;
};
}
