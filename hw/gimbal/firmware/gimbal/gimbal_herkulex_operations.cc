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

#include "gimbal_herkulex_operations.h"

#include "fire_control.h"
#include "gimbal_stabilizer.h"
#include "mahony_imu.h"

GimbalHerkulexOperations::GimbalHerkulexOperations(
    GimbalStabilizer& stabilizer,
    MahonyImu& imu,
    FireControl& fire_control)
    : stabilizer_(stabilizer),
      imu_(imu),
      fire_control_(fire_control) {}

GimbalHerkulexOperations::~GimbalHerkulexOperations() {}

uint8_t GimbalHerkulexOperations::address() const {
  return 98;
}

namespace {
float MakeDegrees(uint32_t value) {
  int32_t signed_value = value;
  if (signed_value >= 0x8000000) {
    signed_value = signed_value - 0x10000000;
  }
  return static_cast<float>(signed_value) / 1000.0;
}
}

void GimbalHerkulexOperations::WriteRam(uint8_t addr, uint8_t val) {
  switch (addr) {
    case 0x34: {
      // torque control
      if (val == 0x60) {
        stabilizer_.SetTorque(true);
      } else {
        stabilizer_.SetTorque(false);
      }
      break;
    }
    case 0x50:
    case 0x54:
    case 0x68: { shadow_ = val & 0x7f; break; }
    case 0x51:
    case 0x55: { shadow_ |= static_cast<uint32_t>(val & 0x7f) << 7; break; }
    case 0x52:
    case 0x56: { shadow_ |= static_cast<uint32_t>(val & 0x7f) << 14; break; }
    case 0x53: {
      shadow_ |= static_cast<uint32_t>(val & 0x7f) << 21;
      const float pitch_deg = MakeDegrees(shadow_);
      stabilizer_.SetImuAttitude(
          pitch_deg, stabilizer_.data().desired_deg.yaw);
      break;
    }
    case 0x57: {
      shadow_ |= static_cast<uint32_t>(val & 0x7f) << 21;
      const float yaw_deg = MakeDegrees(shadow_);
      stabilizer_.SetImuAttitude(
          stabilizer_.data().desired_deg.pitch, yaw_deg);
      break;
    }
    case 0x69: {
      // Magic for setting an absolute yaw position.
      break;
    }
    case 0x6a: {
      if (val != 0) {
        // Do a gyro calibration.
      }
      break;
    }
    case 0x70: { // fire_time
      fire_time_ = val;
      break;
    }
    case 0x71: { // fire_pwm
      fire_control_.SetFire(val, fire_time_);
      break;
    }
    case 0x72: { // agitator_pwm
      fire_control_.SetAgitator(val);
      break;
    }
    case 0x73: {
      fire_control_.SetLaser(val != 0);
      break;
    }
  }
}

uint8_t GimbalHerkulexOperations::ReadRam(uint8_t addr) {
  switch (addr) {
    case 7: { return address(); }
    case 43: { return 0; } // status_error
    case 44: { return 0; } // status_detail
    case 0x34: { return stabilizer_.data().torque_on ? 0x60 : 0x00; }
    case 0x50: { return int_desired_pitch() & 0x7f; }
    case 0x51: { return (int_desired_pitch() >> 7) & 0x7f; }
    case 0x52: { return (int_desired_pitch() >> 14) & 0x7f; }
    case 0x53: { return (int_desired_pitch() >> 21) & 0x7f; }
    case 0x54: { return int_desired_yaw() & 0x7f; }
    case 0x55: { return (int_desired_yaw() >> 7) & 0x7f; }
    case 0x56: { return (int_desired_yaw() >> 14) & 0x7f; }
    case 0x57: { return (int_desired_yaw() >> 21) & 0x7f; }
    case 0x58: { return int_actual_pitch() & 0x7f; }
    case 0x59: { return (int_actual_pitch() >> 7) & 0x7f; }
    case 0x5a: { return (int_actual_pitch() >> 14) & 0x7f; }
    case 0x5b: { return (int_actual_pitch() >> 21) & 0x7f; }
    case 0x5c: { return int_actual_yaw() & 0x7f; }
    case 0x5d: { return (int_actual_yaw() >> 7) & 0x7f; }
    case 0x5e: { return (int_actual_yaw() >> 14) & 0x7f; }
    case 0x5f: { return (int_actual_yaw() >> 21) & 0x7f; }
    case 0x70: { return fire_control_.fire_time_100ms(); }
    case 0x71: { return fire_control_.fire_pwm(); }
    case 0x72: { return fire_control_.agitator_pwm(); }
    case 0x73: { return fire_control_.laser() ? 1 : 0; }
  }
  return 0;
}

void GimbalHerkulexOperations::Reboot() {
}

uint32_t GimbalHerkulexOperations::int_desired_pitch() const {
  return static_cast<int32_t>(
      stabilizer_.data().desired_deg.pitch * 1000.0f);
}

uint32_t GimbalHerkulexOperations::int_desired_yaw() const {
  return static_cast<int32_t>(
      stabilizer_.data().desired_deg.yaw * 1000.0f);
}

uint32_t GimbalHerkulexOperations::int_actual_pitch() const {
  return static_cast<int32_t>(imu_.data().euler_deg.pitch * 1000.0f);
}

uint32_t GimbalHerkulexOperations::int_actual_yaw() const {
  return static_cast<int32_t>(stabilizer_.data().unwrapped_yaw_deg * 1000.0f);
}
