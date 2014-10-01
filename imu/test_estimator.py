#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

import sys
import unittest

sys.path.append('build')
import _estimator
from filter_constant import FilterConstant
import filter_test_cases
import imu_simulator
from run_case import run_case

class EstimatorTest(unittest.TestCase):
    def run_stationary(self, imu_class, filter_class, seed):
        imu = imu_class()
        fc = FilterConstant()
        estimator = filter_class(
            process_noise_gyro=fc.process_noise_gyro,
            process_noise_bias=fc.process_noise_bias,
            initial_noise_attitude=fc.initial_noise_attitude,
            initial_noise_bias=fc.initial_noise_bias,
            measurement_noise_accel=fc.measurement_noise_accel,
            measurement_noise_stationary=fc.measurement_noise_stationary
            )
        case = filter_test_cases.StationaryTest()

        result = run_case(case, imu, estimator,
                          sample_frequency_hz=100.0,
                          test_time=1800.0, seed=seed)
        return result['rms_error']

    def run_seeds(self, imu_class, filter_name, filter_class, expected_error):
        fmt = '%20s %20s %10s %10s'

        error = max(
            [self.run_stationary(imu_class, filter_class, seed)
             for seed in xrange(5)])
        print fmt % (
            imu_class().name,
            filter_name,
            '%.3f' % error,
            '%.3f' % expected_error)

        self.assertLess(error, expected_error)


    def test_miniimuv2_pitch(self):
        self.run_seeds(
            imu_simulator.MiniImuV2,
            'pitch', _estimator.PitchEstimator, 2.0)

    def test_miniimuv2_attitude(self):
        self.run_seeds(
            imu_simulator.MiniImuV2,
            'attitude', _estimator.AttitudeEstimator, 2.0)

    def test_jbimuv2_pitch(self):
        self.run_seeds(
            imu_simulator.JbImuV2,
            'pitch', _estimator.PitchEstimator, 0.4)

    def test_jbimuv2_attitude(self):
        self.run_seeds(
            imu_simulator.JbImuV2,
            'attitude', _estimator.AttitudeEstimator, 0.4)

    def test_max21000_pitch(self):
        self.run_seeds(
            imu_simulator.Max21000,
            'pitch', _estimator.PitchEstimator, 0.1)

    def test_max21000_attitude(self):
        self.run_seeds(
            imu_simulator.Max21000,
            'attitude', _estimator.AttitudeEstimator, 0.1)

    def test_ideal_pitch(self):
        self.run_seeds(
            imu_simulator.IdealImu,
            'pitch', _estimator.PitchEstimator, 0.1)

    def test_ideal_attitude(self):
        self.run_seeds(
            imu_simulator.IdealImu,
            'attitude', _estimator.AttitudeEstimator, 0.1)


if __name__ == '__main__':
    unittest.main()
