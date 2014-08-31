# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import math
import numpy
import os
import sys

sys.path.append(os.path.join(sys.path[0], '../legtool'))

from legtool.tf.quaternion import Quaternion

import ukf_filter

# TODO jpieper: Make this initialized in the class.
IMU_UPDATE_PERIOD = 0.01

class AttitudeEstimator(object):
    # States are:
    # 0: attitude W
    # 1: attitude X
    # 2: attitude Y
    # 3: attitude Z
    # 4: gyro yaw bias
    # 5: gyro pitch bias
    # 6: gyro roll bias

    def state_function(self, state, dt):
        result = state

        this_attitude = Quaternion(
            result[0], result[1], result[2], result[3]).normalized()
        delta = Quaternion()
        for x in self.current_gyro:
            advance = Quaternion.integrate_rotation_rate(
                x.roll + result[4],
                x.pitch + result[5],
                x.yaw + result[6],
                dt)
            delta = delta * advance

        next_attitude = (this_attitude * delta).normalized()
        result[0] = next_attitude.w
        result[1] = next_attitude.x
        result[2] = next_attitude.y
        result[3] = next_attitude.z
        return result

    def attitude(self):
        return Quaternion(*self.ukf.state[0:4,0])

    def gyro_bias(self):
        return list(self.ukf.state[4:7,0])

    def gyro_bias_uncertainty(self):
        return [math.sqrt(x) for x in list(numpy.diag(self.ukf.covariance)[4:7])]

    def __init__(self):
        self.current_gyro = []
        self.init = False
        self.log = open('/tmp/attitude.csv', 'w')
        self.emit_header()
        self.ukf = None

    def emit_header(self):
        self.log.write(','.join(
                ['state_%d' % x for x in range(7)] +
                ['covar_%d_%d' % (x / 7, x % 7) for x in range(7 * 7)]) + '\n')

    def emit_log(self):
        self.log.write(','.join(
                ['%g' % x for x in
                 list(self.state.flatten()) +
                 list(self.covariance.flatten())]) + '\n')

    def process_gyro(self, yaw_rps, pitch_rps, roll_rps):
        self.current_gyro += [
            Euler(yaw=yaw_rps, pitch=pitch_rps, roll=roll_rps)]

    @staticmethod
    def orientation_to_accel(quaternion):
        gravity = numpy.array([0, 0, 1.0])
        expected = quaternion.conjugated().rotate(gravity)
        return numpy.array([[expected[0]],
                            [expected[1]],
                            [expected[2]]])

    @staticmethod
    def accel_measurement(state):
        quaternion = Quaternion(
            state[0,0], state[1,0], state[2,0], state[3,0]).normalized()

        return AttitudeEstimator.orientation_to_accel(quaternion)

    @staticmethod
    def yaw_measurement(state):
        quaternion = Quaternion(
            state[0], state[1], state[2], state[3]).normalized()
        return numpy.array([quaternion.euler().yaw])

    @staticmethod
    def accel_to_orientation(x, y, z):
        roll = math.atan2(-x, z)
        pitch = math.atan2(y, math.sqrt(x**2 + z ** 2))

        quat = Quaternion.from_euler(roll, pitch, 0)
        return quat

    @staticmethod
    def covariance_limit(P):
        for x in range(7):
            if P[x, x] < 1e-9:
                P[x, x] = 1e-9
        for x in range(0, 4):
            if P[x, x] > .15:
                P[x, x] = .15
        for x in range(4, 7):
            if P[x, x] > math.radians(50):
                P[x, x] = math.radians(50)
        return P

    def process_yaw(self, mounting, yaw):
        if self.ukf is None:
            return

        def yaw_meas(state):
            this_attitude = Quaternion(
                state[0], state[1], state[2], state[3]).normalized()
            offset = this_attitude * mounting
            return numpy.array([[offset.euler().yaw]])

        self.ukf.update_measurement(
            numpy.array([[yaw]]),
            measurement_function=yaw_meas,
            measurement_noise=numpy.array([[math.radians(5)]]))

    def process_accel(self, x, y, z):
        # First, normalize.
        norm = math.sqrt(x * x + y * y + z * z)

        x /= norm
        y /= norm
        z /= norm

        # If this is our first update, then initialize the state with
        # the expected attitude based on the accelerometers.
        if not self.init:
            self.init = True

            quat = self.accel_to_orientation(x, y, z)
            state = numpy.array([[quat.w],
                                 [quat.x],
                                 [quat.y],
                                 [quat.z],
                                 [0.],
                                 [0.],
                                 [0.]])
            covariance = numpy.diag([1e-3, 1e-3, 1e-3, 1e-3,
                                     math.radians(1),
                                     math.radians(1),
                                     math.radians(1)])

            self.ukf = ukf_filter.UkfFilter(
                initial_state=state,
                initial_covariance=covariance,
                process_function=self.state_function,
                process_noise=numpy.diag([1e-8, 1e-8, 1e-8, 1e-8,
                                          math.radians(1e-6),
                                          math.radians(1e-6),
                                          math.radians(1e-6)]),
                measurement_function=self.accel_measurement,
                measurement_noise=numpy.diag([10.0, 10.0, 10.0]),
                covariance_limit=self.covariance_limit)

        self.ukf.update_state(IMU_UPDATE_PERIOD)
        self.ukf.update_measurement(numpy.array([[x],[y],[z]]))

        self.current_gyro = []
        #self.emit_log()

class PitchEstimator(object):
    '''A 1 dimensional pitch estimator.  It accepts a gyro input, and
    two accelerometers.

    States are:
      0: pitch (rad)
      1: gyro bias (rad/s)
    '''

    def state_names(self):
        return ['pitch', 'gyro_bias']

    def state_function(self, state, dt):
        result = state

        delta = 0.0
        for x in self.current_gyro:
            delta += (x + result[1]) * dt

        result[0] +=  delta
        return result

    def covariance(self):
        return self.ukf.covariance

    def pitch(self):
        return self.ukf.state[0,0]

    def gyro_bias(self):
        return self.ukf.state[1,0]

    def gyro_bias_uncertainty(self):
        return [math.sqrt(x) for x in list(numpy.diag(self.ukf.covariance)[1])][0]

    def __init__(self, log_filename=None,
                 process_noise_gyro=1e-8,
                 process_noise_bias=math.radians(1e-6),
                 measurement_noise_accel=10):
        self.current_gyro = []
        self.init = False
        if log_filename:
            self.log = open(log_filename, 'w')
        else:
            self.log = None
        self.emit_header()
        self.ukf = None

        self.process_noise_gyro = process_noise_gyro
        self.process_noise_bias = process_noise_bias
        self.measurement_noise_accel = measurement_noise_accel

    def emit_header(self):
        if self.log is None:
            return
        self.log.write(','.join(
                ['state_%d' % x for x in range(2)] +
                ['covar_%d_%d' % (x / 2, x % 2) for x in range(2 * 2)]) + '\n')

    def emit_log(self):
        if self.log is None:
            return
        self.log.write(','.join(
                ['%g' % x for x in
                 list(self.state.flatten()) +
                 list(self.covariance.flatten())]) + '\n')

    def process_gyro(self, pitch_rps):
        self.current_gyro += [ pitch_rps ]

    @staticmethod
    def orientation_to_accel(pitch):
        gravity = numpy.array([0, 0, 1.0])
        expected = Quaternion.from_euler(0., pitch, 0.).conjugated().rotate(gravity)
        return numpy.array([[expected[1]],
                            [expected[2]]])

    @staticmethod
    def accel_measurement(state):
        return PitchEstimator.orientation_to_accel(state[0, 0])

    @staticmethod
    def accel_to_orientation(y, z):
        pitch = math.atan2(y, z)
        return pitch

    @staticmethod
    def covariance_limit(P):
        for x in range(2):
            if P[x, x] < 1e-9:
                P[x, x] = 1e-9
        for x in range(0, 1):
            if P[x, x] > .15:
                P[x, x] = .15
        for x in range(1, 2):
            if P[x, x] > math.radians(50):
                P[x, x] = math.radians(50)
        return P

    def process_accel(self, y, z):
        # First, normalize.
        norm = math.sqrt(y * y + z * z)

        y /= norm
        z /= norm

        # If this is our first update, then initialize the state with
        # the expected attitude based on the accelerometers.
        if not self.init:
            self.init = True

            pitch = self.accel_to_orientation(y, z)
            state = numpy.array([[pitch],
                                 [0.]])
            covariance = numpy.diag([math.radians(5) ** 2,
                                     math.radians(1) ** 2])

            self.ukf = ukf_filter.UkfFilter(
                initial_state=state,
                initial_covariance=covariance,
                process_function=self.state_function,
                process_noise=numpy.diag([self.process_noise_gyro,
                                          self.process_noise_bias]),
                measurement_function=self.accel_measurement,
                measurement_noise=numpy.diag(
                    [self.measurement_noise_accel,
                     self.measurement_noise_accel]),
                covariance_limit=self.covariance_limit)

        self.ukf.update_state(IMU_UPDATE_PERIOD)
        self.ukf.update_measurement(numpy.array([[y],[z]]))

        self.current_gyro = []
        self.emit_log()

