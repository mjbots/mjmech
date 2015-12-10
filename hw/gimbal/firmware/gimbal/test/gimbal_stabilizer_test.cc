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

#include "gimbal/gimbal_stabilizer.h"

#include <boost/test/auto_unit_test.hpp>

#include "gimbal/gpio_pin.h"
#include "gimbal/bldc_pwm.h"

#include "context.h"

namespace {
class GpioPinTest : public GpioPin {
 public:
  virtual ~GpioPinTest() {}
  virtual void Set(bool v) { value_ = v; }
  virtual bool Read() const { return value_; }

  bool value_ = false;
};

class BldcPwmTest : public BldcPwm {
 public:
  virtual ~BldcPwmTest() {}

  virtual void Set(uint16_t phase_a,
                   uint16_t phase_b,
                   uint16_t phase_c) {
    a_ = phase_a;
    b_ = phase_b;
    c_ = phase_c;
  }

  uint16_t a_ = 0;
  uint16_t b_ = 0;
  uint16_t c_ = 0;
};
}

BOOST_AUTO_TEST_CASE(BasicGimbalStabilizerTest) {
  test::Context ctx;

  GpioPinTest motor_enable;
  BldcPwmTest motor1;
  BldcPwmTest motor2;

  AhrsDataSignal ahrs_signal;
  GimbalStabilizer dut(ctx.pool, ctx.clock, ctx.config, ctx.telemetry,
                       ahrs_signal,
                       motor_enable,
                       motor1, motor2);

  BOOST_CHECK_EQUAL(dut.data().state, GimbalStabilizer::kInitializing);
}
