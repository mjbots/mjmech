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

#include "herkulex_protocol.h"

class BldcEncoder;
class FireControl;
class GimbalStabilizer;
class MahonyImu;

class GimbalHerkulexOperations : public HerkulexProtocol::Operations {
 public:
  GimbalHerkulexOperations(GimbalStabilizer&,
                           MahonyImu&,
                           BldcEncoder& yaw_encoder,
                           FireControl&);
  virtual ~GimbalHerkulexOperations();

  uint8_t address() const override;
  void WriteRam(uint8_t addr, uint8_t val) override;
  uint8_t ReadRam(uint8_t addr) override;
  void Reboot() override;

 private:
  uint32_t int_desired_pitch() const;
  uint32_t int_desired_yaw() const;
  uint32_t int_actual_pitch() const;
  uint32_t int_actual_yaw() const;
  uint32_t int_absolute_yaw() const;
  uint32_t int_pitch_rate() const;
  uint32_t int_yaw_rate() const;

  GimbalStabilizer& stabilizer_;
  MahonyImu& imu_;
  BldcEncoder& yaw_encoder_;
  FireControl& fire_control_;

  uint32_t shadow_ = 0;
  uint8_t fire_time_ = 0;
};
