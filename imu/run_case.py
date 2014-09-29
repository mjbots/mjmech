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

import math
import numpy

import imu_simulator

G = 9.81

def run_case(test_case, imu_parameters, estimator,
             sample_frequency_hz, test_time, seed=0):
    imusim = imu_simulator.ImuSimulator(
        imu_parameters, sample_frequency_hz, seed)

    estimator.process_measurement(0., 0., 0.,
                                  0., 0., 1.0)

    NUM_COUNT = int(test_time * sample_frequency_hz)
    error = 0.0
    diff = 0.0

    # Measure the noise for N seconds and set that as the bias for
    # each channel.
    INIT_COUNT = int(2 * sample_frequency_hz)
    gyro_x_bias, gyro_y_bias, gyro_z_bias = 0., 0., 0.
    for count in range(INIT_COUNT):
        imusim.update_reality(0.0, 0.0, G,
                              0., 0., 0.)
        gyro_x_bias += imusim.measured_gyro_x_rps
        gyro_y_bias += imusim.measured_gyro_y_rps
        gyro_z_bias += imusim.measured_gyro_z_rps

    gyro_x_bias /= INIT_COUNT
    gyro_y_bias /= INIT_COUNT
    gyro_z_bias /= INIT_COUNT
    estimator.set_initial_gyro_bias(yaw_rps=-gyro_z_bias,
                                    pitch_rps=-gyro_x_bias,
                                    roll_rps=-gyro_y_bias)

    for count in range(NUM_COUNT):
        test_case.advance(1.0 / sample_frequency_hz)
        true_accel_y, true_accel_z = test_case.accel_yz_mps2()

        imusim.update_reality(0.0, true_accel_y, true_accel_z,
                              test_case.gyro(), 0., 0.)

        accel_x = imusim.measured_accel_x_mps2
        accel_y = imusim.measured_accel_y_mps2
        accel_z = imusim.measured_accel_z_mps2
        gyro_x = imusim.measured_gyro_x_rps
        gyro_y = imusim.measured_gyro_y_rps
        gyro_z = imusim.measured_gyro_z_rps

        estimator.process_measurement(
            yaw_rps=gyro_z, pitch_rps=gyro_x, roll_rps=gyro_y,
            x_g = accel_x, y_g = accel_y, z_g = accel_z)

        diff = estimator.pitch_error(test_case.expected_pitch())
        error += diff ** 2

    diag = estimator.covariance_diag()

    return { 'rms_error': math.degrees(math.sqrt((error / NUM_COUNT))),
             'uncert': numpy.sqrt(diag) }
