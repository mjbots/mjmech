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

#include "gimbal/as5048_driver.h"

#include <boost/test/auto_unit_test.hpp>

#include "async_i2c_simulator.h"
#include "context.h"

namespace {
class As5048Simulator : public test::AsyncI2CSimulator {
 public:
  As5048Simulator() : test::AsyncI2CSimulator(0x40) {}
  virtual ~As5048Simulator() {}

  uint8_t Read(uint8_t address) override {
    switch (address) {
      case 250: { return 0x3e; }
      case 251: { return 0x01; }
      case 252: { return 0x87; }
      case 253: { return 0x31; }
      case 254: { return 0xc6; }
      case 255: { return 0x23; }
    }
    return 0;
  };

  void Write(uint8_t address, uint8_t value) override {}
};
}

BOOST_AUTO_TEST_CASE(BasicAs5048DriverTest) {
  test::Context ctx;
  As5048Simulator dut_sim;

  As5048Driver dut(ctx.pool, gsl::ensure_z("dut"),
                   &dut_sim, nullptr, ctx.clock, ctx.config, ctx.telemetry);

  ctx.clock.value_ = 1234;
  As5048Driver::Data data;
  int count = 0;
  int error = 0;
  dut.AsyncRead(&data, [&](int err) { count += 1; error = err; });

  while (true) {
    if (!dut_sim.ProcessAction()) { break; }
  }

  BOOST_CHECK_EQUAL(count, 1);
  BOOST_CHECK_EQUAL(error, 0);
  BOOST_CHECK_EQUAL(data.timestamp, 1234);
  BOOST_CHECK_EQUAL(data.agc, 0x3e);
  BOOST_CHECK_EQUAL(data.diagnostics, 0x01);
  BOOST_CHECK_EQUAL(data.magnitude, 0x21f1);
  BOOST_CHECK_EQUAL(data.angle, 0x31a3);
}
