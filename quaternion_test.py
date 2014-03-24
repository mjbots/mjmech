#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

from math import degrees
from math import radians
from numpy import array
import sys
import unittest

from quaternion import Quaternion

class QuaternionTest(unittest.TestCase):
    def check_vector_close(self, vector, x, y, z):
        self.assertAlmostEqual(vector[0], x)
        self.assertAlmostEqual(vector[1], y)
        self.assertAlmostEqual(vector[2], z)

    def test_basic(self):
        v = array([10, 0, 0])

        v = Quaternion.from_euler(0, 0, radians(90)).rotate(v)
        self.check_vector_close(v, 0, -10.0, 0)

        v = Quaternion.from_euler(0, 0, radians(-90)).rotate(v)
        self.check_vector_close(v, 10, 0, 0)

        v = Quaternion.from_euler(0, radians(90), 0).rotate(v)
        self.check_vector_close(v, 10, 0, 0)

        v = Quaternion.from_euler(radians(90), 0, 0).rotate(v)
        self.check_vector_close(v, 0, 0, -10)

        v = Quaternion.from_euler(0, 0, radians(90)).rotate(v)
        self.check_vector_close(v, 0, 0, -10)

        v = Quaternion.from_euler(0, radians(90), 0).rotate(v)
        self.check_vector_close(v, 0, 10, 0)

        v = Quaternion.from_euler(radians(90), 0, 0).rotate(v)
        self.check_vector_close(v, 0, 10, 0)

        v = Quaternion.from_euler(0, 0, radians(90)).rotate(v)
        self.check_vector_close(v, 10, 0, 0)

    def check_euler(self, euler, roll_rad, pitch_rad, yaw_rad):
        self.assertAlmostEqual(degrees(euler.yaw),
                               degrees(yaw_rad))
        self.assertAlmostEqual(degrees(euler.pitch),
                               degrees(pitch_rad))
        self.assertAlmostEqual(degrees(euler.roll),
                               degrees(roll_rad))

    def test_euler_and_back(self):
        tests = [ (45, 0, 0),
                  (0, 45, 0),
                  (0, 0, 45),
                  (0, 90, 0),
                  (0, 90, 20),
                  (0, -90, 0),
                  (0, -90, -10),
                  (0, -90, 30),
                  (10, 20, 30),
                  (-30, 10, 20), ]
        for test in tests:
            try:
                self.check_euler(
                    Quaternion.from_euler(radians(test[0]),
                                          radians(test[1]),
                                          radians(test[2])).euler(),
                    radians(test[0]),
                    radians(test[1]),
                    radians(test[2]))
            except:
                print >> sys.stderr, 'in test:', test
                raise

    def test_multiplication(self):
        x90 = Quaternion.from_euler(0, radians(90), 0)
        xn90 = Quaternion.from_euler(0, -radians(90), 0)
        y90 = Quaternion.from_euler(radians(90), 0, 0)

        result = xn90 * y90 * x90
        vector = array([0, 1, 0])
        vector = result.rotate(vector)
        self.check_vector_close(vector, 1, 0, 0)

        initial = Quaternion.from_euler(0, 0, radians(45))
        initial = Quaternion.from_euler(0, 0, radians(45)) * initial
        self.check_euler(initial.euler(), 0, 0, radians(90))

        initial = Quaternion.from_euler(0, radians(10), 0) * initial
        vector = initial.rotate(vector)
        self.check_vector_close(vector, 0, -0.9848078, -0.17364818)
        self.check_euler(initial.euler(), radians(10), 0, radians(90))

    def test_multiplication2(self):
        attitude = Quaternion.from_euler(radians(-5), 0, radians(90))
        attitude = attitude * Quaternion.from_euler(0, 0, radians(90))
        self.check_euler(attitude.euler(), 0, radians(5), radians(180))

    def test_multiplication3(self):
        attitude = Quaternion.from_euler(radians(-3), radians(3), 0)
        attitude = attitude * Quaternion.integrate_rotation_rate(
            0, 0, radians(-5), 1)
        self.check_euler(attitude.euler(),
                         radians(-3.24974326),
                         radians(2.727438544),
                         radians(-4.99563857))

if __name__ == '__main__':
    unittest.main()
