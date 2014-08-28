#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

'''Simulate a 1d pitch estimator in order to determine optimal UKF
filter gains.'''

import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy
import pylab
import random
import scipy
import scipy.integrate

import ahrs
import imu_error_model

SAMPLE_FREQUENCY_HZ = 100.0
G = 9.81

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
        self.velocity_rad_s = 6.2
        self.time_s = 0.
        self.accelx_mps2 = 0.
        self.accely_mps2 = 0.
        self.velx_mps = 0.
        self.vely_mps = 0.

    def gyro(self):
        return self.velocity_rad_s

    def accel_yz(self):
        # The gravity component.
        x = 0.0
        y = 1.0

        # The change in velocity component.
        x += self.accelx_mps2 / G
        y += self.accely_mps2 / G

        # TODO jpieper: Need to rotate everything into the IMU
        # reference frame.
        y, z = x, y

        return y, z

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

def plot_pendulum():
    pc = PendulumCase()
    results = []
    for i in range(800):
        results.append((pc.expected_pitch(),
                        pc.gyro(),
                        pc.accelx_mps2 / G,
                        pc.accely_mps2 / G,
                        pc.accel_yz()))
        pc.advance(0.01)

    pylab.plot([x[0] for x in results], label='pitch_rad')
    pylab.plot([x[1] for x in results], label='gyro_rps')
    pylab.plot([x[2] for x in results], label='accelx_g')
    pylab.plot([x[3] for x in results], label='accely_g')
    pylab.plot([x[4][0] for x in results], label='accel_y')
    pylab.plot([x[4][1] for x in results], label='accel_z')
    pylab.legend()
    pylab.show()

def animate_pendulum():
    pc = PendulumCase()
    fig = plt.figure()
    fig.subplots_adjust(left=0., right=1., bottom=0., top=1.)
    ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                         xlim=(-3., 3.), ylim=(-3., 3.))

    pend_line, = ax.plot([], [], '-', lw=3)
    vel_line, = ax.plot([], [], 'b-', lw=5)
    acc_line, = ax.plot([], [], 'r-', lw=5)

    def animate(i):
        pc.advance(0.01)

        px = math.sin(pc.pitch_rad)
        py = -math.cos(pc.pitch_rad)

        pend_line.set_data([0., px],
                           [0., py])
        vel_line.set_data([px, px + 0.2 * pc.velx_mps],
                          [py, py + 0.2 * pc.vely_mps])
        acc_line.set_data([px, px + 0.05 * pc.accelx_mps2],
                          [py, py + 0.05 * pc.accely_mps2])

    ani = animation.FuncAnimation(fig, animate, frames=600, interval=10)
    plt.show()


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

    print "rms error:", math.degrees(math.sqrt((error / NUM_COUNT)))
    diag = numpy.diag(estimator.covariance())
    print "pitch uncert:", math.degrees(math.sqrt(diag[0]))
    print "bias uncert:", math.degrees(math.sqrt(diag[1]))

if __name__ == '__main__':
    #main()
    animate_pendulum()

