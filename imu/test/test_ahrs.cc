// Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

#include "ahrs.h"

#include <boost/test/auto_unit_test.hpp>

BOOST_AUTO_TEST_CASE(BasicPitchEstimator) {
  imu::PitchEstimator estimator(0.0008 * 0.0008,
                                0.0512 * 0.0512,
                                1.0 * 1.0);

  for (int i = 0; i < 1000; i++) {
    estimator.ProcessGyro(0.0);
    estimator.ProcessAccel(0.4794, 0.8776);
  }

  BOOST_CHECK_SMALL(estimator.pitch_rad() - 0.5, 1e-2);
}
