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
import sys

sys.path.append(os.path.join(sys.path[0], '../legtool'))

from legtool.tf.quaternion import Quaternion

import imu_error_model
import ukf_filter

# TODO jpieper: Make this initialized in the class.
IMU_UPDATE_PERIOD = 0.01

class Euler(object):
    def __init__(self, yaw=None, pitch=None, roll=None):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

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
                x.roll + result[6],
                x.pitch + result[5],
                x.yaw + result[4],
                dt)
            delta = delta * advance

        next_attitude = (this_attitude * delta).normalized()

        result[0] = next_attitude.w
        result[1] = next_attitude.x
        result[2] = next_attitude.y
        result[3] = next_attitude.z
        return result

    def set_initial_gyro_bias(self, yaw_rps, pitch_rps, roll_rps):
        self.ukf.state[4, 0] = yaw_rps
        self.ukf.state[5, 0] = pitch_rps
        self.ukf.state[6, 0] = roll_rps

    def attitude(self):
        return Quaternion(*self.ukf.state[0:4,0]).normalized()

    def pitch_error(self, pitch):
        return (self.attitude() *
                Quaternion.from_euler(
                0., pitch, 0.).conjugated()).euler().pitch

    def pitch(self):
        return self.attitude().euler().pitch

    def gyro_bias(self):
        return list(self.ukf.state[4:7,0])

    def gyro_bias_uncertainty(self):
        return [math.sqrt(x) for x in list(numpy.diag(self.ukf.covariance)[4:7])]

    def covariance_diag(self):
        return numpy.diag(self.ukf.covariance)

    def state_names(self):
        return ['w', 'x', 'y', 'z', 'gx', 'gy', 'gz']

    def __init__(self,
                 process_noise_gyro,
                 process_noise_bias,
                 measurement_noise_accel,
                 initial_noise_attitude,
                 initial_noise_bias,
                 log_filename="/tmp/rotate.log",
                 extra_log=None):
        self.process_noise_gyro = process_noise_gyro
        self.process_noise_bias = process_noise_bias
        self.measurement_noise_accel = measurement_noise_accel
        self.initial_noise_attitude = initial_noise_attitude
        self.initial_noise_bias = initial_noise_bias
        self.current_gyro = []
        self.init = False
        if log_filename:
            self.log = open(log_filename, 'w')
        else:
            self.log = None
        self.emit_header(extra_log)
        self.ukf = None

    def emit_header(self, extra_log):
        if self.log is None:
            return
        if extra_log is None:
            extra_log = []
        self.log.write(','.join(
                [x[0] for x in extra_log] +
                ['gyro_x', 'gyro_y', 'gyro_z', 'accel_x', 'accel_y', 'accel_z'] +
                ['roll', 'pitch', 'yaw'] +
                ['state_%d' % x for x in range(7)] +
                ['covar_%d_%d' % (x / 7, x % 7) for x in range(7 * 7)]) + '\n')

    def emit_log(self, gyro, accel, extra_log):
        if self.log is None:
            return
        e = self.attitude().euler()
        if extra_log is None:
            extra_log = []
        self.log.write(','.join(
                ['%g' % x for x in
                 [x[1] for x in extra_log] +
                 [gyro.pitch, gyro.roll, gyro.yaw] +
                 [accel[0], accel[1], accel[2]] +
                 [e.roll, e.pitch, e.yaw] +
                 list(self.ukf.state.flatten()) +
                 list(self.ukf.covariance.flatten())]) + '\n')

    def process_gyro(self, yaw_rps, pitch_rps, roll_rps):
        e = Euler(yaw=yaw_rps, pitch=pitch_rps, roll=roll_rps)
        self.current_gyro += [ e ]

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

    def process_accel(self, x, y, z, extra_log=None):
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
            covariance = numpy.diag([self.initial_noise_attitude] * 4 +
                                    [self.initial_noise_bias] * 3)

            self.ukf = ukf_filter.UkfFilter(
                initial_state=state,
                initial_covariance=covariance,
                process_function=self.state_function,
                process_noise=numpy.diag([self.process_noise_gyro] * 4 +
                                         [self.process_noise_bias] * 3),
                measurement_function=self.accel_measurement,
                measurement_noise=numpy.diag([self.measurement_noise_accel] * 3),
                covariance_limit=self.covariance_limit)

        self.ukf.update_state(IMU_UPDATE_PERIOD)

        for g in self.current_gyro:
            extra = Euler(0., 0., 0.)
            if abs(g.yaw) > 0.95 * math.radians(250):
                extra.yaw = math.radians(2000.0)
            if abs(g.roll) > 0.95 * math.radians(250):
                extra.roll = math.radians(2000.0)
            if abs(g.pitch) > 0.95 * math.radians(250):
                extra.pitch = math.radians(2000.0)
            if extra.yaw or extra.roll or extra.pitch:
                delta = Quaternion.integrate_rotation_rate(
                    extra.roll, extra.pitch, extra.yaw, IMU_UPDATE_PERIOD)
                ta = self.attitude()
                na = (ta * delta).normalized()
                da = Quaternion(na.w - ta.w,
                                na.x - ta.x,
                                na.y - ta.y,
                                na.z - ta.z)
                self.ukf.covariance += numpy.diag(
                    [da.w, da.x, da.y, da.z, 0., 0., 0.]) ** 2

        self.ukf.update_measurement(numpy.array([[x],[y],[z]]),
                                    mahal_limit=None)

        gyro = Euler(0., 0., 0.)
        if len(self.current_gyro):
            gyro = self.current_gyro[0]
        self.emit_log(gyro, (x, y, z), extra_log)
        self.current_gyro = []

class Dummy(object):
    def __init__(self):
        self.value = None

    def sample(self):
        result = self.value
        self.value = None
        return result

class LowpassFilter(object):
    def __init__(self, frequency):
        self.dummy = Dummy()
        self.filter = imu_error_model.ChebyshevFilter(
            self.dummy,
            1.0 / IMU_UPDATE_PERIOD,
            frequency,
            imu_error_model.ChebyshevFilter.LOW_PASS,
            5,
            4)

    def __call__(self, value):
        self.dummy.value = value
        return self.filter.sample()

class PitchEstimator(object):
    '''A 1 dimensional pitch estimator.  It accepts a gyro input, and
    two accelerometers.

    States are:
      0: pitch (rad)
      1: gyro bias (rad/s)
    '''

    def state_names(self):
        return ['pitch', 'gyro_bias']

    def set_initial_gyro_bias(self, yaw_rps, pitch_rps, roll_rps):
        self.ukf.state[1, 0] = pitch_rps

    def state_function(self, state, dt):
        result = state

        delta = 0.0
        for x in self.current_gyro:
            delta += (x + result[1]) * dt

        result[0] +=  delta
        return result

    def covariance(self):
        return self.ukf.covariance

    def covariance_diag(self):
        return list(numpy.diag(self.ukf.covariance))

    def pitch(self):
        return self.ukf.state[0,0]

    def gyro_bias(self):
        return self.ukf.state[1,0]

    def attitude(self):
        return Quaternion.from_euler(0., self.pitch(), 0.)

    def pitch_error(self, pitch):
        return (self.attitude() *
                Quaternion.from_euler(
                0., pitch, 0.).conjugated()).euler().pitch

    def gyro_bias_uncertainty(self):
        return [math.sqrt(x) for x in list(numpy.diag(self.ukf.covariance)[1])][0]

    def __init__(self,
                 process_noise_gyro,
                 process_noise_bias,
                 measurement_noise_accel,
                 initial_noise_attitude,
                 initial_noise_bias,
                 accel_filter=None,
                 log_filename=None,
                 extra_log=None):
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
        self.initial_noise_attitude = initial_noise_attitude
        self.initial_noise_bias = initial_noise_bias
        if accel_filter is None:
            self.accel_y_filter = lambda x: x
            self.accel_z_filter = lambda x: x
        else:
            self.accel_y_filter = LowpassFilter(accel_filter)
            self.accel_z_filter = LowpassFilter(accel_filter)


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

    def process_gyro(self, yaw_rps, pitch_rps, roll_rps):
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
#        for x in range(0, 1):
#            if P[x, x] > .15:
#                P[x, x] = .15
#        for x in range(1, 2):
#            if P[x, x] > math.radians(50):
#                P[x, x] = math.radians(50)
        return P

    def process_accel(self, x, y, z):
        # Filter
        y = self.accel_y_filter(y)
        z = self.accel_z_filter(z)

        # Normalize
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
            covariance = numpy.diag([self.initial_noise_attitude,
                                     self.initial_noise_bias])

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
