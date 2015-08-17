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

#include "servo_interface.h"

namespace legtool {
template <typename Servo>
class HerkuleXServoInterface : public ServoInterface {
 public:
  typedef HerkuleXConstants HC;

  HerkuleXServoInterface(Servo* servo) : servo_(servo) {}
  Servo* servo() { return servo_; }
  const Servo* servo() const { return servo_; }

  struct Parameters {
    double pose_time_s = 0.1;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(LT_NVP(pose_time_s));
    }
  };

  Parameters* parameters() { return &parameters_; }

  virtual void SetPose(const std::vector<Joint>& joints,
                       ErrorHandler handler) override {
    struct Target {
      uint8_t address;
      uint16_t position;
      uint8_t leds;
    };
    std::vector<Target> targets(joints.size());
    for (const auto& joint: joints) {
      targets.emplace_back(Target{
          MapAddress(joint.address), AngleToCount(joint.angle_deg), 0});
    }
    servo_->SJog(targets, parameters_.pose_time_s, handler);
  }

  virtual void EnablePower(PowerState power_state,
                           const std::vector<int>& ids_in,
                           ErrorHandler handler) override {
    uint8_t value = [power_state]() {
      switch (power_state) {
        case kPowerFree: { return 0x00; }
        case kPowerBrake: { return 0x40; }
        case kPowerEnable: { return 0x60; }
      }
      BOOST_ASSERT(false);
    }();

    std::vector<int> ids(ids_in);
    if (ids.empty()) { ids.push_back(Servo::BROADCAST); }

    HandlePowerAck(boost::system::error_code(), ids, value, handler);
  }

  void HandlePowerAck(const boost::system::error_code& ec,
                      const std::vector<int>& ids,
                      uint8_t value,
                      ErrorHandler handler) {
    if (ec) { handler(ec); return; }

    if (ids.empty()) {
      handler(boost::system::error_code());
      return;
    }

    std::vector<int> new_ids = ids;
    int to_send = new_ids.back();
    new_ids.pop_back();

    servo_->RamWrite(to_send, HC::torque_control(), value,
                     std::bind(&HerkuleXServoInterface::HandlePowerAck, this,
                               std::placeholders::_1,
                               new_ids, value, handler));
  }

  virtual void GetPose(
      const std::vector<int>& ids, PoseHandler handler) override {
    DoGetPose(boost::system::error_code(), 0,
              boost::none,
              ids, std::vector<Joint>{}, handler);
  }

  void DoGetPose(const boost::system::error_code& ec,
                 int value,
                 boost::optional<int> address,
                 const std::vector<int>& ids,
                 const std::vector<Joint>& current_result,
                 PoseHandler handler) {
    if (ec) { handler(ec, {}); return; }
    if (ids.empty()) { handler(ec, current_result); return; }
    std::vector<Joint> result;
    if (address) {
      result.emplace_back(Joint{*address, CountsToAngleDeg(value)});
    }

    int to_send = ids.back();
    std::vector<int> new_ids = ids;
    new_ids.pop_back();
    servo_->RamRead(to_send, HC::position(),
                    std::bind(&HerkuleXServoInterface::DoGetPose, this,
                              std::placeholders::_1,
                              std::placeholders::_2,
                              to_send,
                              new_ids, result, handler));
  }

  virtual void GetTemperature(
      const std::vector<int>& ids, TemperatureHandler handler) override {
    throw std::runtime_error("not implemented");
    // std::vector<Temperature> result;
    // for (int address: ids) {
    //   result.emplace_back(Temperature{address, CountsToTemperatureC(
    //           servo_->RamRead(address, HC::temperature_c(), yield))});
    // }
    // return result;
  }

  virtual void GetVoltage(
      const std::vector<int>& ids, VoltageHandler handler) override {
    throw std::runtime_error("not implemented");

    // std::vector<Voltage> result;
    // for (int address: ids) {
    //   result.emplace_back(Voltage{address, CountsToVoltage(
    //           servo_->RamRead(address, HC::voltage(), yield))});
    // }
    // return result;
  }

  uint16_t AngleToCount(double angle_deg) const {
    return std::min(
        1023, std::max(0, static_cast<int>(512 + angle_deg / 0.325)));
  }

  double CountsToAngleDeg(int counts) const {
    return (counts - 512) * 0.325;
  }

  double CountsToTemperatureC(int counts) const {
    // Note, this formula was derived from the Dongbu lookup table,
    // and becomes terribly inaccurate below -20C.
    return (counts - 40) * 0.5125 - 19.38;
  }

  double CountsToVoltage(int counts) const {
    return counts * 0.074;
  }

 private:
  static uint8_t MapAddress(int address) {
    if (address < 0 || address > 0xfe) {
      throw std::runtime_error("invalid address");
    }
    return static_cast<uint8_t>(address);
  }

  Parameters parameters_;
  Servo* const servo_;
};
}
