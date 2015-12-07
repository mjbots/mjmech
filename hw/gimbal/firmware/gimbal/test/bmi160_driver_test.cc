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

#include "gimbal/bmi160_driver.h"

#include <map>

#include <boost/test/auto_unit_test.hpp>

#include "gimbal/async_i2c.h"
#include "gimbal/clock.h"
#include "gimbal/lock_manager.h"
#include "gimbal/persistent_config.h"
#include "gimbal/telemetry_manager.h"

#include "async_i2c_simulator.h"
#include "clock_test.h"
#include "flash_test.h"
#include "stream_test.h"

namespace {

class Bmi160Simulator : public test::AsyncI2CSimulator {
 public:
  Bmi160Simulator() : test::AsyncI2CSimulator(0xd0) {}
  virtual ~Bmi160Simulator() {}

  uint8_t Read(uint8_t address) override {
    switch (address) {
      case 0x00: { return 0xd1; }
      default: { return register_file_[address]; }
    }
    return 0;
  }

  void Write(uint8_t address, uint8_t value) override {
    register_file_[address] = value;
  }

  std::map<int, int> register_file_;
};

}

BOOST_AUTO_TEST_CASE(Bmi160DriverTest) {
  SizedPool<> pool;
  Bmi160Simulator imu_sim;
  test::ClockTest clock;
  test::FlashTest flash;
  test::TestWriteStream test_stream;
  LockManager lock_manager;
  PersistentConfig config(pool, flash, test_stream);
  TelemetryManager telemetry(pool, test_stream, lock_manager);

  Bmi160Driver dut(pool, gsl::ensure_z("dut"),
                   imu_sim, clock, config, telemetry);

  int count = 0;
  int error = 0;
  dut.AsyncStart([&](int err) { count++; error = err; });

  BOOST_CHECK_EQUAL(count, 0);

  // Our very first thing should have asked for the chip ID.
  BOOST_CHECK_EQUAL(imu_sim.addresses_read_.count(0x00), 1);

  bool acc_poweron = false;
  bool gyr_poweron = false;

  std::set<Bmi160Driver::State> observed_states;

  // Process callbacks until the initialization sequence is completed.
  for (int i = 0; i < 10000 && count == 0; i++) {
    observed_states.insert(dut.data()->state);

    if (i % 2) {
      dut.Poll();
    } else {
      if (!imu_sim.ProcessAction()) {
        clock.value_ += 1;
      }
    }
    auto cmd = imu_sim.register_file_[0x7e];
    if (cmd != 0) {
      if (cmd == 0x11) { acc_poweron = true; }
      else if (cmd == 0x15) { gyr_poweron = true; }
      else {
        BOOST_REQUIRE_EQUAL(cmd, 0);
      }
      imu_sim.register_file_[0x7e] = 0;
    }
  }

  BOOST_CHECK_EQUAL(count, 1);
  BOOST_CHECK_EQUAL(error, 0);

  BOOST_CHECK_EQUAL(acc_poweron, true);
  BOOST_CHECK_EQUAL(gyr_poweron, true);

  typedef Bmi160Driver::State BS;
  BOOST_CHECK_EQUAL(observed_states.count(BS::kInitial), 0);
  BOOST_CHECK_EQUAL(observed_states.count(BS::kIdentifying), 1);
  BOOST_CHECK_EQUAL(observed_states.count(BS::kConfiguring), 1);
  BOOST_CHECK_EQUAL(observed_states.count(BS::kPoweringAccel), 1);
  BOOST_CHECK_EQUAL(observed_states.count(BS::kPoweringGyro), 1);
  BOOST_CHECK_EQUAL(observed_states.count(BS::kErrorCheck), 1);
  BOOST_CHECK_EQUAL(observed_states.count(BS::kOperational), 0);

  BOOST_CHECK_EQUAL(dut.data()->state, BS::kOperational);
}
