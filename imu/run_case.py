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
import os

import imu_simulator
import plot_accumulator

G = 9.81

def run_case(test_case, imu_parameters, estimator,
             sample_frequency_hz, test_time, seed=0):
    if 'IMU_TRACE' in os.environ:
        fname = os.environ['IMU_TRACE']
        my_name = fname
        count = 0
        while os.path.exists(my_name):
            count += 1
            my_name = fname + '.%d' % count

        pa = plot_accumulator.PlotAccumulator(my_name)
    else:
        pa = plot_accumulator.NullAccumulator()

    imusim = imu_simulator.ImuSimulator(
        imu_parameters, sample_frequency_hz, seed)

    NUM_COUNT = int(test_time * sample_frequency_hz)
    error = 0.0
    diff = 0.0

    # Measure the noise for N seconds and set that as the bias for
    # each channel.
    INIT_COUNT = int(2 * sample_frequency_hz)
    gyro_x_bias, gyro_y_bias, gyro_z_bias = 0., 0., 0.
    accel_x_avg, accel_y_avg, accel_z_avg = 0., 0., 0.
    for count in range(INIT_COUNT):
        true_accel_y, true_accel_z = test_case.accel_yz_mps2()
        imusim.update_reality(0.0, true_accel_y, true_accel_z,
                              0., 0., 0.)
        gyro_x_bias += imusim.measured_gyro_x_rps
        gyro_y_bias += imusim.measured_gyro_y_rps
        gyro_z_bias += imusim.measured_gyro_z_rps

        accel_x_avg += imusim.measured_accel_x_mps2
        accel_y_avg += imusim.measured_accel_y_mps2
        accel_z_avg += imusim.measured_accel_z_mps2

    gyro_x_bias /= INIT_COUNT
    gyro_y_bias /= INIT_COUNT
    gyro_z_bias /= INIT_COUNT

    accel_x_avg /= INIT_COUNT
    accel_y_avg /= INIT_COUNT
    accel_z_avg /= INIT_COUNT

    estimator.set_initial_gyro_bias(yaw_rps=-gyro_z_bias,
                                    pitch_rps=-gyro_x_bias,
                                    roll_rps=-gyro_y_bias)
    estimator.set_initial_accel(x_g=accel_x_avg/G,
                                y_g=accel_y_avg/G,
                                z_g=accel_z_avg/G)

    for count in range(NUM_COUNT):
        pa.advance_time(count / float(sample_frequency_hz))

        test_case.advance(1.0 / sample_frequency_hz)
        true_accel_y, true_accel_z = test_case.accel_yz_mps2()
        pa.add(true_accel_x_mps2=0.,
               true_accel_y_mps2=true_accel_y,
               true_accel_z_mps2=true_accel_z)
        pa.add(true_gyro_x_dps=math.degrees(test_case.gyro()),
               true_gyro_y_dps=0.,
               true_gyro_z_dps=0.)

        imusim.update_reality(0.0, true_accel_y, true_accel_z,
                              test_case.gyro(), 0., 0.)

        accel_x = imusim.measured_accel_x_mps2
        accel_y = imusim.measured_accel_y_mps2
        accel_z = imusim.measured_accel_z_mps2
        gyro_x = imusim.measured_gyro_x_rps
        gyro_y = imusim.measured_gyro_y_rps
        gyro_z = imusim.measured_gyro_z_rps

        pa.add(accel_x_mps2=accel_x,
               accel_y_mps2=accel_y,
               accel_z_mps2=accel_z)

        pa.add(gyro_x_dps=math.degrees(gyro_x),
               gyro_y_dps=math.degrees(gyro_y),
               gyro_z_dps=math.degrees(gyro_z))

        estimator.process_measurement(
            yaw_rps=gyro_z, pitch_rps=gyro_x, roll_rps=gyro_y,
            x_g=accel_x/G, y_g=accel_y/G, z_g=accel_z/G)

        pa.add(true_yaw_deg=0.,
               true_pitch_deg=math.degrees(test_case.expected_pitch()),
               true_roll_deg=0.)

        pa.add(yaw_deg=math.degrees(estimator.yaw()),
               pitch_deg=math.degrees(estimator.pitch()),
               roll_deg=math.degrees(estimator.roll()),
               yaw_dps=math.degrees(estimator.yaw_rps()),
               pitch_dps=math.degrees(estimator.pitch_rps()),
               roll_dps=math.degrees(estimator.roll_rps()))

        diff = estimator.pitch_error(test_case.expected_pitch())
        error += diff ** 2

    diag = estimator.covariance_diag()

    return { 'rms_error': math.degrees(math.sqrt((error / NUM_COUNT))),
             'uncert': numpy.sqrt(diag) }
