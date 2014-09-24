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

'''Simulate a 1d pitch estimator in order to determine optimal UKF
filter gains.'''

import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy
import optparse
import pylab
import random
import re
import scipy
import scipy.integrate
import sys

import estimator
from estimator import Quaternion
import imu_error_model
from filter_test_cases import TestCase, StationaryTest
import imu_simulator
from imu_simulator import MiniImuV2, Max21000, JbImuV2, IdealImu

sys.path.append('build')
import _estimator

SAMPLE_FREQUENCY_HZ = 100.0
G = 9.81
TEST_CASE_TIME = 50.0

def rotate(vec, val):
    s = math.sin(val)
    c = math.cos(val)

    return (vec[0] * c - vec[1] * s,
            vec[0] * s + vec[1] * c)

class RotatingTest(TestCase):
    def __init__(self, noise_level=2.0, frequency=1.0):
        self.pitch_rad = 0.0
        self.gyro_rad_s = 0.0

        self.noise_level = noise_level
        self.return_coeff = 0.05

        self.rng = random.Random(0)
        self.white_noise = imu_error_model.WhiteNoise(
            self.rng, self.noise_level)
        self.noise = imu_error_model.ChebyshevFilter(
            self.white_noise, SAMPLE_FREQUENCY_HZ, frequency,
            imu_error_model.ChebyshevFilter.LOW_PASS,
            5.0, 4)

    def name(self):
        return 'rotate %.1f' % self.noise_level

    def advance(self, dt):
        self.gyro_rad_s = (self.noise.sample() -
                           self.return_coeff * self.pitch_rad)

        self.pitch_rad += self.gyro_rad_s * dt

    def gyro(self):
        return self.gyro_rad_s

    def accel_yz_mps2(self):
        x = 0.0
        y = 1.0

        y, z = rotate((x, y), -self.pitch_rad)

        return y * G, z * G

    def expected_pitch(self):
        return self.pitch_rad

    def pos(self):
        return 0., 0.

class BouncingTest(RotatingTest):
    def __init__(self, accel_noise=5.0, accel_freq=10.0, damp_vel=1.0,
                 **kwargs):
        super(BouncingTest, self).__init__(**kwargs)

        self.accel_noise = accel_noise
        self.posx_m = 0.0
        self.posy_m = 0.0
        self.velx_mps = 0.0
        self.vely_mps = 0.0
        self.accelx_mps2 = 0.0
        self.accely_mps2 = 0.0

        self.damp_vel = damp_vel
        self.damp_pos = 1.0

        self.accelx_white_noise = imu_error_model.WhiteNoise(
            self.rng, accel_noise)
        self.accelx_noise = imu_error_model.ChebyshevFilter(
            self.accelx_white_noise, SAMPLE_FREQUENCY_HZ, accel_freq,
            imu_error_model.ChebyshevFilter.LOW_PASS,
            5.0, 4)

        self.accely_white_noise = imu_error_model.WhiteNoise(
            self.rng, accel_noise)
        self.accely_noise = imu_error_model.ChebyshevFilter(
            self.accely_white_noise, SAMPLE_FREQUENCY_HZ, accel_freq,
            imu_error_model.ChebyshevFilter.LOW_PASS,
            5.0, 4)

    def name(self):
        return 'bounce %.1f %.1f %.1f' % (
            self.noise_level, self.accel_noise, self.damp_vel)

    def advance(self, dt):
        super(BouncingTest, self).advance(dt)

        self.accelx_mps2 = (self.accelx_noise.sample() -
                            self.damp_vel * self.velx_mps -
                            self.damp_pos * self.posx_m)
        self.accely_mps2 = (self.accely_noise.sample() -
                            self.damp_vel * self.vely_mps -
                            self.damp_pos * self.posy_m)

        self.posx_m += self.velx_mps * dt + 0.5 * self.accelx_mps2 * (dt ** 2)
        self.posy_m += self.vely_mps * dt + 0.5 * self.accely_mps2 * (dt ** 2)
        self.velx_mps += self.accelx_mps2 * dt
        self.vely_mps += self.accely_mps2 * dt

    def accel_yz_mps2(self):
        # The gravity component.
        x = 0.0
        y = 1.0

        # The change in velocity component.
        x += self.accelx_mps2 / G
        y += self.accely_mps2 / G

        y, z = rotate((x, y), -self.pitch_rad)

        return y * G, z * G

    def pos(self):
        return self.posx_m, self.posy_m


class PendulumTest(TestCase):
    def __init__(self, v=1.2):
        self.length_m = 1.0
        self.pitch_rad = 0.0
        self.initial_velocity_rad_s = self.velocity_rad_s = v
        self.time_s = 0.
        self.accelx_mps2 = 0.
        self.accely_mps2 = 0.
        self.velx_mps = 0.
        self.vely_mps = 0.

    def name(self):
        return 'pendulum v=%.1f' % self.initial_velocity_rad_s

    def gyro(self):
        return self.velocity_rad_s

    def accel_yz_mps2(self):
        # The gravity component.
        x = 0.0
        y = 1.0

        # The change in velocity component.
        x += self.accelx_mps2 / G
        y += self.accely_mps2 / G

        y, z = rotate((x, y), -self.pitch_rad)

        return y * G, z * G

    def dydt(self, y, t):
        theta, theta_dot = y
        theta_dot_dot = (-G / self.length_m) * scipy.sin(theta)
        return [theta_dot, theta_dot_dot]

    def advance(self, dt):
        y = [self.pitch_rad, self.velocity_rad_s]
        y_traj = scipy.integrate.odeint(
            self.dydt, y,
            scipy.array(
                [self.time_s,
                 self.time_s + dt]))
        new_pitch_rad, new_velocity_rad_s = y_traj[-1]

        oldvx = self.velocity_rad_s * math.cos(self.pitch_rad)
        newvx = new_velocity_rad_s * math.cos(new_pitch_rad)
        self.velx_mps = newvx
        self.accelx_mps2 = (newvx - oldvx) / dt

        oldvy = self.velocity_rad_s * math.sin(self.pitch_rad)
        newvy = new_velocity_rad_s * math.sin(new_pitch_rad)
        self.vely_mps = newvy
        self.accely_mps2 = (newvy - oldvy) / dt

        self.pitch_rad, self.velocity_rad_s = new_pitch_rad, new_velocity_rad_s
        self.time_s += dt

    def expected_pitch(self):
        return self.pitch_rad

    def pos(self):
        return (math.sin(self.pitch_rad) * self.length_m,
                -self.length_m * math.cos(self.pitch_rad))


def plot_pendulum():
    pc = PendulumTest()
    results = []
    for i in range(800):
        results.append((pc.expected_pitch(),
                        pc.gyro(),
                        pc.accelx_mps2 / G,
                        pc.accely_mps2 / G,
                        pc.accel_yz_mps2()))
        pc.advance(0.01)

    pylab.plot([x[0] for x in results], label='pitch_rad')
    pylab.plot([x[1] for x in results], label='gyro_rps')
    pylab.plot([x[2] for x in results], label='accelx_g')
    pylab.plot([x[3] for x in results], label='accely_g')
    pylab.plot([x[4][0] for x in results], label='accel_y')
    pylab.plot([x[4][1] for x in results], label='accel_z')
    pylab.legend()
    pylab.show()

def animate_case(pc):
    fig = plt.figure()
    fig.subplots_adjust(left=0., right=1., bottom=0., top=1.)
    ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                         xlim=(-3., 3.), ylim=(-3., 3.))

    ax.plot([0], [0], '+')
    pend_line, = ax.plot([], [], '-', lw=3)
    vel_line, = ax.plot([], [], 'b-', lw=5)
    acc_line, = ax.plot([], [], 'r-', lw=5)
    y_line, = ax.plot([], [], 'y-', lw=3)
    z_line, = ax.plot([], [], 'g-', lw=3)
    g_line, = ax.plot([], [], 'r-', lw=3)

    def animate(i):
        pc.advance(0.01)

        px, py = pc.pos()

        pend_line.set_data([0., px],
                           [0., py])
#        vel_line.set_data([px, px + 0.1 * pc.velx_mps],
#                          [py, py + 0.1 * pc.vely_mps])
#        ax = px + pc.accelx_mps2 / G
#        ay = py + pc.accely_mps2 / G
#        acc_line.set_data([px, ax], [py, ay])
#        g_line.set_data(
#            [ax, ax],
#            [ay, ay + 1.0])

        ay, az = pc.accel_yz_mps2()
        ay /= G
        az /= G
        y_line.set_data([px, px + ay * math.cos(pc.pitch_rad)],
                        [py, py + ay * math.sin(pc.pitch_rad)])
        z_line.set_data([px, px + az * math.sin(-pc.pitch_rad)],
                        [py, py + az * math.cos(-pc.pitch_rad)])

    ani = animation.FuncAnimation(fig, animate, frames=600, interval=10)
    plt.show()


def run_case(options, test_case, imu_parameters, estimator):
    imusim = imu_simulator.ImuSimulator(
        imu_parameters, SAMPLE_FREQUENCY_HZ, options.seed)

    if options.attitude:
        estimator.process_accel(0., 0., 1.0)
    else:
        estimator.process_accel(0, 1.0)

    NUM_COUNT = int(TEST_CASE_TIME * SAMPLE_FREQUENCY_HZ)
    error = 0.0
    diff = 0.0

    # Measure the noise for N seconds and set that as the bias for
    # each channel.
    INIT_COUNT = int(2 * SAMPLE_FREQUENCY_HZ)
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
        test_case.advance(1.0 / SAMPLE_FREQUENCY_HZ)
        true_accel_y, true_accel_z = test_case.accel_yz_mps2()

        imusim.update_reality(0.0, true_accel_y, true_accel_z,
                              test_case.gyro(), 0., 0.)

        accel_x = imusim.measured_accel_x_mps2
        accel_y = imusim.measured_accel_y_mps2
        accel_z = imusim.measured_accel_z_mps2
        gyro_x = imusim.measured_gyro_x_rps
        gyro_y = imusim.measured_gyro_y_rps
        gyro_z = imusim.measured_gyro_z_rps

        if options.attitude:
            estimator.process_gyro(
                yaw_rps=gyro_z, pitch_rps=gyro_x, roll_rps=gyro_y)
            extra_log = [('true_pitch', test_case.expected_pitch()),
                         ('pitch_err', diff),
                         ]
            kwargs = {}
            if options.python:
                kwargs['extra_log'] = extra_log
            estimator.process_accel(
                accel_x, accel_y, accel_z,
                **kwargs)
        else:
            estimator.process_gyro(gyro_x)
            estimator.process_accel(accel_y, accel_z)

        diff = estimator.pitch_error(test_case.expected_pitch())
        error += diff ** 2

    diag = estimator.covariance_diag()

    return { 'rms_error': math.degrees(math.sqrt((error / NUM_COUNT))),
             'uncert': numpy.sqrt(diag) }

def run(options):

    cases = []
    cases += [lambda: BouncingTest(damp_vel=0.1)]
    cases += [lambda: BouncingTest()]
    cases += [lambda: BouncingTest(noise_level=5.0, accel_noise=20.)]
    cases += [lambda: RotatingTest()]
    cases += [lambda: RotatingTest(noise_level=5)]
    cases += [lambda: RotatingTest(noise_level=10)]
    cases += [lambda: RotatingTest(noise_level=15)]
    cases += [lambda: StationaryTest()]
    cases += [lambda: PendulumTest(v=0.5)]
    cases += [lambda: PendulumTest(v=1.0)]
    cases += [lambda: PendulumTest(v=2.0)]
    cases += [lambda: PendulumTest(v=4.0)]

    imus = [MiniImuV2(), JbImuV2(), Max21000(), IdealImu()]

    if len(options.limit):
        def match(case):
            for exp in options.limit:
                if re.search(exp, x().name()):
                    return True
            return False

        cases = [x for x in cases if match(x)]

    if options.animate:
        print 'Animating:', cases[0].name()
        animate_case(cases[0])
        return

    print '%20s %s' % ('test_case', ' '.join('%12s' % x.name for x in imus))

    for case_gen in cases:
        case = case_gen()
        print '%20s' % case.name(),

        for imu in imus:
            case = case_gen()
            kwargs = {}
            extra_log = [('true_pitch', 0),
                         ('pitch_err', 0),
                         ('true_gyro_x_bias', 0),
                         ('true_gyro_y_bias', 0),
                         ('true_gyro_z_bias', 0)]

            if options.python:
                module = estimator
                kwargs['extra_log'] = extra_log
            else:
                module = _estimator

            if not options.attitude:
                estimator_class = module.PitchEstimator
            else:
                estimator_class = module.AttitudeEstimator

            my_estimator = estimator_class(
                process_noise_gyro=math.radians(0.0002)**2,
                process_noise_bias=math.radians(0.0256)**2,
                measurement_noise_accel=4.0**2,
                initial_noise_attitude=1e-3,
                initial_noise_bias=1e-6,
                **kwargs)

            result = run_case(options, case, imu, my_estimator)

            covar = ' '.join('%s=%f' % (x, math.degrees(y))
                             for x, y in zip(my_estimator.state_names(),
                                             result['uncert']))
            if len(covar) > 40:
                covar = covar[0:40]
            print '%12s' % ('%.4f' % result['rms_error']),
        print

def main():
    parser = optparse.OptionParser()
    parser.add_option('-l', '--limit', action='append', default=[],
                      help='Only consider tests matching these regular exps')
    parser.add_option('--animate', action='store_true', default=False,
                      help='Animate the first selected test')
    parser.add_option('-p', '--python', action='store_true', default=False,
                      help='Use python filter')
    parser.add_option('-a', '--attitude', action='store_true', default=False,
                      help='Use full 2d attitude filter')
    parser.add_option('-s', '--seed', type='int', default=0)

    options, args = parser.parse_args()
    assert len(args) == 0
    run(options)

if __name__ == '__main__':
    main()
