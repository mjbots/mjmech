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
#include <set>

#include <boost/test/auto_unit_test.hpp>

#include "gimbal/async_i2c.h"
#include "gimbal/clock.h"
#include "gimbal/lock_manager.h"
#include "gimbal/persistent_config.h"
#include "gimbal/telemetry_manager.h"

#include "flash_test.h"
#include "stream_test.h"

namespace {
class ClockTest : public Clock {
 public:
  virtual ~ClockTest() {}

  virtual uint32_t timestamp() const override { return value_; }

  uint32_t value_ = 0;
};

class Bmi160Simulator : public AsyncI2C {
 public:
  virtual ~Bmi160Simulator() {}

  virtual void AsyncRead(uint8_t device_address,
                         uint8_t memory_address,
                         const gsl::string_span& buffer,
                         ErrorCallback callback) override {
    BOOST_CHECK_EQUAL(device_address, 0xd0);

    for (std::size_t i = 0; i < buffer.size(); i++) {
      *(buffer.data() + i) = static_cast<char>(Read(memory_address + i));
    }

    read_callback_ = callback;
  }

  virtual void AsyncWrite(uint8_t device_address,
                          uint8_t memory_address,
                          const gsl::cstring_span& buffer,
                          ErrorCallback callback) override {
    BOOST_CHECK_EQUAL(device_address, 0xd0);

    for (std::size_t i = 0; i < buffer.size(); i++) {
      Write(memory_address + i, static_cast<uint8_t>(*(buffer.data() + i)));
    }

    write_callback_ = callback;
  }

  uint8_t Read(int address) {
    addresses_read_.insert(address);
    switch (address) {
      case 0x00: { return 0xd1; }
      default: { return register_file_[address]; }
    }
    return 0;
  }

  void Write(int address, uint8_t value) {
    addresses_written_.insert(address);
    register_file_[address] = value;
  }

  void ProcessAction(ClockTest* clock) {
    if (!write_callback_.valid() &&
        !read_callback_.valid()) {
      // Just advance time by a tick.
      clock->value_++;
      return;
    }
    BOOST_CHECK_EQUAL(write_callback_.valid() ^ read_callback_.valid(), true);

    if (read_callback_.valid()) {
      auto callback = read_callback_;
      read_callback_ = ErrorCallback();
      callback(0);
    } else {
      auto callback = write_callback_;
      write_callback_ = ErrorCallback();
      callback(0);
    }
  }

  void ProcessWrite() {
    BOOST_CHECK(!write_callback_.valid());
    BOOST_CHECK_EQUAL(read_callback_.valid(), true);

    auto callback = read_callback_;
    read_callback_ = ErrorCallback();
    callback(0);
  }

  std::set<int> addresses_read_;
  std::set<int> addresses_written_;
  std::map<int, int> register_file_;
  ErrorCallback read_callback_;
  ErrorCallback write_callback_;
};

}

BOOST_AUTO_TEST_CASE(Bmi160DriverTest) {
  SizedPool<> pool;
  Bmi160Simulator imu_sim;
  ClockTest clock;
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
      imu_sim.ProcessAction(&clock);
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
