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
import random

import imu_error_model

G = 9.81

class IdealImu(object):
    name = 'ideal'

    accel_noise = 0.0
    accel_bias_stability = 0.0
    accel_max_mps2 = G * 100

    gyro_noise = 0.0
    gyro_bias_stability = 0.0

    def __init__(self, gyro_max_rps=math.radians(2000)):
        self.gyro_max_rps = gyro_max_rps

class MiniImuV2(object):
    name = 'miniimuv2'

    accel_noise = 1.9e-2
    accel_bias_stability = 1.2e-3
    accel_max_mps2 = G * 8

    gyro_noise = 5.3e-4
    gyro_bias_stability = 1.4e-4

    def __init__(self, gyro_max_rps=math.radians(1000)):
        self.gyro_max_rps = gyro_max_rps

class Max21000(object):
    name = 'max21000'

    accel_noise = 8.3e-4
    accel_bias_stability = 1.4e-4
    accel_max_mps2 = G * 8

    gyro_noise = 2.2e-5
    gyro_bias_stability = 8.4e-6

    def __init__(self, gyro_max_rps=math.radians(1000)):
        self.gyro_max_rps = gyro_max_rps

class JbImuV2(object):
    name = 'jbimuv2'

    accel_noise = 1.5e-3
    accel_bias_stability = 2e-4
    accel_max_mps2 = G * 8

    gyro_noise = 3.0e-4
    gyro_bias_stability = 6.5e-5

    def __init__(self, gyro_max_rps=math.radians(1000)):
        self.gyro_max_rps = gyro_max_rps

def limit(value, top):
    assert top > 0.

    if value > top:
        return top
    if value < -top:
        return -top

    return value

class ImuSimulator(object):
    def __init__(self, model, sample_rate_hz, seed=0):
        self.model = model
        self.rng = random.Random(seed)

        self.accel_x_error, self.accel_y_error, self.accel_z_error = [
            imu_error_model.InertialErrorModel(
                self.rng,
                sample_rate_hz,
                model.accel_noise,
                model.accel_bias_stability,
                0.0)
            for x in range(3)]

        self.gyro_x_error, self.gyro_y_error, self.gyro_z_error = [
            imu_error_model.InertialErrorModel(
                self.rng,
                sample_rate_hz,
                model.gyro_noise,
                model.gyro_bias_stability,
                0.0)
            for x in range(3)]

    def update_reality(self, accel_x_mps2, accel_y_mps2, accel_z_mps2,
                       gyro_x_rps, gyro_y_rps, gyro_z_rps):
        self.accel_x_mps2 = accel_x_mps2
        self.accel_y_mps2 = accel_y_mps2
        self.accel_z_mps2 = accel_z_mps2

        self.gyro_x_rps = gyro_x_rps
        self.gyro_y_rps = gyro_y_rps
        self.gyro_z_rps = gyro_z_rps

        self.measured_accel_x_mps2 = limit(
            self.accel_x_error.sample() + self.accel_x_mps2,
            self.model.accel_max_mps2)
        self.measured_accel_y_mps2 = limit(
            self.accel_y_error.sample() + self.accel_y_mps2,
            self.model.accel_max_mps2)
        self.measured_accel_z_mps2 = limit(
            self.accel_z_error.sample() + self.accel_z_mps2,
            self.model.accel_max_mps2)

        self.measured_gyro_x_rps = limit(
            self.gyro_x_error.sample() + self.gyro_x_rps,
            self.model.gyro_max_rps)
        self.measured_gyro_y_rps = limit(
            self.gyro_y_error.sample() + self.gyro_y_rps,
            self.model.gyro_max_rps)
        self.measured_gyro_z_rps = limit(
            self.gyro_z_error.sample() + self.gyro_z_rps,
            self.model.gyro_max_rps)
