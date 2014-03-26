#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import numpy
from numpy import array
import unittest

import ukf_filter

class UkfFilterTest(unittest.TestCase):
    def test_basic(self):
        def test_process(x, dt_s):
            return x + array(
                [[0],
                 [x[0,0] * dt_s],
                 [x[1,0] * dt_s + 0.5 * x[0,0] * dt_s ** 2]])

        def test_measurement(x):
            return array([[x[2, 0]]])

        dut = ukf_filter.UkfFilter(
            initial_state=array([[0.2], [0.0], [0.0]]),
            initial_covariance=numpy.diag([1.0, 2.0, 3.0]),
            process_function=test_process,
            process_noise=numpy.diag([0.1, 0.1, 0.1]),
            measurement_function = test_measurement,
            measurement_noise=array([[2.0]]))

        meas = 0.5
        for x in range(200):
            meas += 0.5
            dut.update_state(0.1)
            dut.update_measurement(array([[meas]]))

        self.assertAlmostEqual(round(dut.state[2, 0], 2), meas)
        self.assertAlmostEqual(round(dut.state[1, 0], 2), 0.5 / 0.1)

if __name__ == '__main__':
    unittest.main()
