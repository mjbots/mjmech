#!/usr/bin/python

# Copyright 2012-2014 Josh Pieper, jjp@pobox.com.  All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import imu_error_model
import unittest

class FilterTest(unittest.TestCase):
    def assertArraysClose(self, actual, expected):
        self.assertTrue(
            max([abs(a - b) for a, b in zip(actual, expected)]) < 0.0001,
            msg='actual=%s expected=%s' % (actual, expected))

    def test_chebyshev(self):
        result = imu_error_model.ChebyshevFilter._calculate_stage(
            1, 0.1, imu_error_model.ChebyshevFilter.LOW_PASS, 0, 4)

        expected_a = [ 0.061885, 0.123770, 0.061885 ]
        expected_b = [ 1.048600, -0.296140 ]
        self.assertArraysClose(result.a, expected_a)
        self.assertArraysClose(result.b[1:], expected_b)

        result = imu_error_model.ChebyshevFilter._calculate_stage(
            2, 0.1, imu_error_model.ChebyshevFilter.HIGH_PASS, 10, 4)

        expected_a = [ 0.922919, -1.845840, 0.922919 ]
        expected_b = [ 1.446913, -0.836653 ]
        self.assertArraysClose(result.a, expected_a)
        self.assertArraysClose(result.b[1:], expected_b)

if __name__ == '__main__':
    unittest.main()
