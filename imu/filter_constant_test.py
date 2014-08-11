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

def main():
    rng = random.Random()

    # These constants are from the Pololu MiniIMU v2 @2g.
    accel_y_error, accel_z_error = [
        imu_error_model.InertialErrorModel(
            rng,
            SAMPLE_FREQUENCY_HZ,
            5.21e-3,
            9.65e-5,
            2.47e-5)
        for x in range(2)]

    # These constants are from the Pololu MiniIMU v2 at 2000dps
    # maximum rate.
    gyro_error = imu_error_model.InertialErrorModel(
        rng,
        SAMPLE_FREQUENCY_HZ,
        math.radians(0.0241),
        math.radians(0.00808),
        1.38e-5)

    estimator = ahrs.PitchEstimator(process_noise_gyro=math.radians(0.001)**2,
                                    process_noise_bias=math.radians(0.01)**2,
                                    measurement_noise_accel=0.1**2)
    estimator.process_accel(0, 1.0)

    NUM_COUNT = 200000
    error = 0.0

    for count in range(NUM_COUNT):
        accel_y = accel_y_error.sample()
        accel_z = accel_z_error.sample() + 1.0
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

