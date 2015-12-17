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

#include "gimbal/fire_control.h"

#include <boost/test/auto_unit_test.hpp>

#include "gimbal/gpio_pin.h"
#include "gimbal/pwm_pin.h"

#include "context.h"

namespace {
class TestGpio : public GpioPin {
 public:
  virtual ~TestGpio() {}
  void Set(bool v) override { value_ = v; }
  bool Read() const override { return value_; }
  bool value_ = false;
};

class TestPwm : public PwmPin {
 public:
  virtual ~TestPwm() {}

  void Set(uint16_t v) override { value_ = v; }
  uint16_t value_ = 0;
};
}

BOOST_AUTO_TEST_CASE(BasicFireControlTest) {
  test::Context ctx;

  TestGpio laser_enable;
  TestGpio pwm_enable;
  TestPwm aeg_pwm;
  TestPwm agitator_pwm;
  FireControl dut(ctx.pool, ctx.clock, ctx.config, ctx.telemetry,
                  laser_enable, pwm_enable, aeg_pwm, agitator_pwm);
}
