#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

'''Simulate a 1d pitch estimator in order to determine optimal UKF
filter gains.'''

import math
import numpy
import random

import ahrs
import imu_error_model

SAMPLE_FREQUENCY_HZ = 100.0

class TestCase(object):
    def advance(self, dt):
        raise NotImplementedError

    def gyro(self):
        raise NotImplementedError

    def accel_yz(self):
        raise NotImplementedError

    def expected_pitch(self):
        raise NotImplementedError

class StationaryTest(TestCase):
    def advance(self, dt):
        pass

    def gyro(self):
        return 0.0

    def accel_yz(self):
        return 0., 1.

    def expected_pitch(self):
        return 0.

class PendulumCase(TestCase):
    def __init__(self):
        self.length_m = 1.0
        self.pitch_rad = 0.0
        self.velocity_rad_s = 1.0

    def gyro(self):
        return self.velocity_rad_s

    def accel_yz(self):
        # The gravity component.
        y = 1.0 * math.sin(self.pitch_rad)
        z = 1.0 * math.cos(self.pitch_rad)

        # The change in velocity component.


    def advance(self, dt):
        # TODO jpieper
        raise NotImplementedError

    def expected_pitch(self):
        return self.pitch_rad


def main():
    rng = random.Random()

    # These constants are from the Pololu MiniIMU v2 @8g.
    accel_y_error, accel_z_error = [
        imu_error_model.InertialErrorModel(
            rng,
            SAMPLE_FREQUENCY_HZ,
            1.9e-2,
            1.2e-3,
            1.3e-5)
        for x in range(2)]

    # These constants are from the Pololu MiniIMU v2 at 2000dps
    # maximum rate.
    gyro_error = imu_error_model.InertialErrorModel(
        rng,
        SAMPLE_FREQUENCY_HZ,
        math.radians(0.001),  # 5.3e-4 rad/sqrt(s), 0.0303 deg/sqrt(s)
        math.radians(0.001),  # 1.4e-4 rad/s, 0.0080 deg/s
        1.38e-5)               # 3.6e-6 rad/sqrt(s)

    estimator = ahrs.PitchEstimator(process_noise_gyro=math.radians(0.05)**2,
                                    process_noise_bias=math.radians(0.0512)**2,
                                    measurement_noise_accel=0.05**2)
    estimator.process_accel(0, 1.0)

    NUM_COUNT = 100000
    error = 0.0

    for count in range(NUM_COUNT):
        accel_y = accel_y_error.sample() / 9.81
        accel_z = (accel_z_error.sample() / 9.81) + 1.0
        gyro = gyro_error.sample()

        estimator.process_gyro(gyro)
        estimator.process_accel(accel_y, accel_z)

        error += estimator.pitch() ** 2
#        print accel_y, accel_z, gyro, estimator.pitch()

    print "rms error:", math.degrees(math.sqrt((error / NUM_COUNT)))
    diag = numpy.diag(estimator.covariance())
    print "pitch uncert:", math.degrees(math.sqrt(diag[0]))
    print "bias uncert:", math.degrees(math.sqrt(diag[1]))

if __name__ == '__main__':
    main()

