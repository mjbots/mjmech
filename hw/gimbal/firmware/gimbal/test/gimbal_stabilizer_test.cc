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

  GpioPinTest boost_enable;
  GpioPinTest motor_enable;
  BldcPwmTest motor1;
  BldcPwmTest motor2;
  GpioPinTest torque_led;

  AhrsData ahrs_data;
  ahrs_data.rate_hz = 100;
  AhrsDataSignal ahrs_signal;
  GimbalStabilizer dut(ctx.pool, ctx.clock, ctx.config, ctx.telemetry,
                       ahrs_signal,
                       boost_enable,
                       motor_enable,
                       motor1, motor2,
                       torque_led);

  for (auto command: {
      "set gimbal.pitch.power 1.0",
          "set gimbal.yaw.power 1.0",
          "set gimbal.pitch.motor 1",
          "set gimbal.yaw.motor 2",
          }) {
    ctx.config.Command(gsl::ensure_z(command),
                       CommandManager::Response(&ctx.test_stream, [](int){}));
  }

  BOOST_CHECK_EQUAL(dut.data().state, GimbalStabilizer::kInitializing);

  ctx.clock.value_ = 1000;
  const auto advance = [&]() {
    ctx.clock.value_ += 100;
    ahrs_data.timestamp = ctx.clock.timestamp();
    ahrs_signal(&ahrs_data);
  };

  // Move forward 2s in 10ms steps.  This should get us initialized.
  for (int i = 0; i < 199; i++) {
    advance();
  }

  BOOST_CHECK_EQUAL(dut.data().state, GimbalStabilizer::kOperating);

  // Since the reported AHRS is exactly our default command, the
  // output commands should be right in the middle.
  BOOST_CHECK_EQUAL(motor1.a_, 32767);
  BOOST_CHECK_NE(motor1.b_, 32767);
  BOOST_CHECK_NE(motor1.c_, 32767);
  BOOST_CHECK_EQUAL(motor2.a_, 32767);
  BOOST_CHECK_NE(motor2.b_, 32767);
  BOOST_CHECK_NE(motor2.c_, 32767);

  // But the motor should not be enabled until we tell it to be.
  BOOST_CHECK_EQUAL(motor_enable.value_, false);

  dut.SetTorque(true);
  BOOST_CHECK_EQUAL(motor_enable.value_, false);

  advance();
  BOOST_CHECK_EQUAL(motor_enable.value_, true);

  // Now if the pitch is a little bit high, we should see our command
  // start to ramp down to correct.
  ahrs_data.euler_deg.pitch = 0.1;
  advance();
  BOOST_CHECK_EQUAL(motor1.a_, 28639);
  BOOST_CHECK_EQUAL(motor2.a_, 32767);

  advance();
  BOOST_CHECK_EQUAL(motor1.a_, 28619);
  BOOST_CHECK_EQUAL(motor2.a_, 32767);

  // And when we get back, we stop changing.
  ahrs_data.euler_deg.pitch = 0.0;

  advance();
  BOOST_CHECK_EQUAL(motor1.a_, 32725);
  BOOST_CHECK_EQUAL(motor2.a_, 32767);

  advance();
  BOOST_CHECK_EQUAL(motor1.a_, 32725);
  BOOST_CHECK_EQUAL(motor2.a_, 32767);

  // Finally, verify that a yaw disturbance results in any change at
  // all.
  ahrs_data.euler_deg.yaw = 0.1;
  advance();
  BOOST_CHECK_EQUAL(motor1.a_, 32725);
  BOOST_CHECK_EQUAL(motor2.a_, 28639);
}
