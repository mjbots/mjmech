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

#include "gimbal/lock_manager.h"
#include "gimbal/persistent_config.h"
#include "gimbal/telemetry_manager.h"

#include "async_i2c_simulator.h"
#include "clock_test.h"
#include "flash_test.h"
#include "stream_test.h"

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
  SizedPool<> pool;
  As5048Simulator dut_sim;
  test::ClockTest clock;
  test::FlashTest flash;
  test::TestWriteStream test_stream;
  LockManager lock_manager;
  PersistentConfig config(pool, flash, test_stream);
  TelemetryManager telemetry(pool, test_stream, lock_manager);

  As5048Driver dut(pool, gsl::ensure_z("dut"),
                   &dut_sim, nullptr, clock, config, telemetry);

  clock.value_ = 1234;
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
