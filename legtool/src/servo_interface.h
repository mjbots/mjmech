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

namespace legtool {
class ServoInterface : boost::noncopyable {
 public:
  struct Joint {
    int address;
    double angle_deg;
  };
  virtual void SetPose(const std::vector<Joint>&,
                       boost::asio::yield_context) = 0;

  enum PowerState {
    kPowerFree,
    kPowerBrake,
    kPowerEnable,
  };
  virtual void EnablePower(PowerState power_state, const std::vector<int>&,
                           boost::asio::yield_context) = 0;

  virtual std::vector<Joint> GetPose(const std::vector<int>&,
                                     boost::asio::yield_context) = 0;

  struct Temperature {
    int address;
    double temperature_C;
  };
  
  virtual std::vector<Temperature> GetTemperature(
      const std::vector<int>&,
      boost::asio::yield_context) = 0;

  struct Voltage {
    int address;
    double voltage;
  };
  
  virtual std::vector<Voltage> GetVoltage(const std::vector<int>&,
                                          boost::asio::yield_context) = 0;
};

}
