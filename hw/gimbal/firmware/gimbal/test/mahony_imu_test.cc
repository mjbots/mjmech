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

#include "gimbal/mahony_imu.h"

#include <boost/test/auto_unit_test.hpp>

#include "context.h"

BOOST_AUTO_TEST_CASE(BasicMahonyImuTest) {
  test::Context ctx;

  ImuDataSignal imu_signal;
  MahonyImu dut(ctx.pool, ctx.clock, ctx.config, ctx.telemetry, imu_signal);
  AhrsData ahrs_data;
  int count = 0;
  dut.data_signal()->Connect([&](const AhrsData* data) {
      ahrs_data = *data;
      count++;
    });

  ImuData data;
  data.rate_hz = 100;
  data.accel_g.z = 1.0f;
  BOOST_CHECK_EQUAL(count, 0);
  imu_signal(&data);
  BOOST_CHECK_EQUAL(count, 1);

  // Now try to pick an acceleration that indicates a slight pitch.
  // Verify that we converge to that correct pitch with time.
  data.accel_g.z = std::cos(0.1);
  data.accel_g.y = std::sin(0.1);

  for (int i = 0; i < 10000; i++) {
    imu_signal(&data);
  }

  BOOST_CHECK_SMALL(ahrs_data.euler_deg.pitch - 5.73, 1e-2);
  BOOST_CHECK_SMALL(ahrs_data.euler_deg.roll - 0.0, 1e-2);

  // Now try with some amount of roll.

  data.accel_g.y = 0.0;
  data.accel_g.x = std::sin(0.1);

  for (int i = 0; i < 10000; i++) {
    imu_signal(&data);
  }

  BOOST_CHECK_SMALL(ahrs_data.euler_deg.pitch - 0.0, 1e-2);
  BOOST_CHECK_SMALL(ahrs_data.euler_deg.roll + 5.73, 1e-2);

  // If we start seeing some gyro about the roll axis, this roll value
  // should increase for a bit, then assuming we have an integrative
  // term, should stabilize back where it was.
  data.gyro_dps.y = 2.0;

  for (int i = 0; i < 10; i++) {
    imu_signal(&data);
  }

  BOOST_CHECK_GE(ahrs_data.euler_deg.roll, -5.63);

  for (int i = 0; i < 20000; i++) {
    imu_signal(&data);
  }

  BOOST_CHECK_SMALL(ahrs_data.euler_deg.roll + 5.73, 1e-2);
}
