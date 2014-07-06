# Copyright 2014 Josh Pieper, jjp@pobox.com.
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

'''Implementation of Quaternions using an x-right, y-forward, z-up
coordinate system.'''


import collections
import math
import numpy

Euler = collections.namedtuple('Euler', 'roll pitch yaw')

class Quaternion(object):
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return '%f %f %f %f' % (self.w, self.x, self.y, self.z)

    def rotate(self, vector3d):
        p = Quaternion(0.0, vector3d[0], vector3d[1], vector3d[2])
        q = self * p * self.conjugated()

        if isinstance(vector3d, numpy.ndarray):
            return numpy.array([q.x, q.y, q.z])
        return vector3d.__class__([q.x, q.y, q.z])

    def conjugated(self):
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def normalized(self):
        norm = math.sqrt(self.w ** 2 + self.x ** 2 + self.y ** 2 + self.z ** 2)
        return Quaternion(self.w / norm,
                          self.x / norm,
                          self.y / norm,
                          self.z / norm)

    def euler(self):
        sp = 2 * (self.w * self.x + self.y * self.z)
        if abs(sp - 1.0) < 1e-8: # north pole
            pitch = math.radians(90)
            roll = 0
            yaw = -math.atan2((self.w * self.y + self.x * self.z),
                              -(self.y * self.z - self.w * self.x))
        elif abs(sp + 1.0) < 1e-8: # south pole
            pitch = math.radians(-90)
            roll = 0
            yaw = math.atan2((self.w * self.y + self.x * self.z),
                             (self.y * self.z - self.w * self.x))
        else:
            pitch = math.asin(sp)
            roll = -math.atan2(2 * (self.x * self.z - self.w * self.y),
                               1.0 - 2 * self.x ** 2 - 2 * self.y ** 2)
            yaw = math.atan2(2 * (self.x * self.y - self.w * self.z),
                             1 - 2 * self.x ** 2 - 2 * self.z ** 2)

        return Euler(roll, pitch, yaw)

    def matrix(self):
        return numpy.array(
            [[ self.w ** 2 + self.x ** 2 - self.y ** 2 - self.z ** 2,
               2 * (self.x * self.y - self.w * self.z),
               2 * (self.w * self.y + self.x * self.z) ],
             [ 2 * (self.x * self.y + self.w * self.z),
               self.w ** 2 - self.x ** 2 + self.y ** 2 - self.z ** 2,
               2 * (self.y * self.z - self.w * self.x) ],
             [ 2 * (self.x * self.z - self.w * self.y),
               2 * (self.w * self.x + self.y * self.z),
               self.w ** 2 - self.x ** 2 - self.y ** 2 + self.z ** 2]])

    def map_to_frame(self, vector3d):
        return self.conjugated().rotate(vector3d)

    def map_from_frame(self, vector3d):
        return self.rotate(vector3d)

    def __mul__(self, other):
        a = self.w
        b = self.x
        c = self.y
        d = self.z

        e = other.w
        f = other.x
        g = other.y
        h = other.z

        return Quaternion(a * e - b * f - c * g - d * h,
                          b * e + a * f + c * h - d * g,
                          a * g - b * h + c * e + d * f,
                          a * h + b * g - c * f + d * e)

    def copy(self):
        return Quaternion(self.w, self.x, self.y, self.z)

    @staticmethod
    def from_euler(roll_rad, pitch_rad, yaw_rad):
        # Quaternions multiply in opposite order, and we want to get
        # into roll, pitch, then yaw standard.
        return (Quaternion.from_axis_angle(yaw_rad, 0, 0, -1) *
                Quaternion.from_axis_angle(pitch_rad, 1, 0, 0) *
                Quaternion.from_axis_angle(roll_rad, 0, 1, 0))

    @staticmethod
    def from_axis_angle(angle_rad, x, y, z):
        c = math.cos(angle_rad / 2.)
        s = math.sin(angle_rad / 2.)

        return Quaternion(c, x * s, y * s, z * s)

    @staticmethod
    def integrate_rotation_rate(roll_rate_rps, pitch_rate_rps, yaw_rate_rps,
                                dt_s):
        # This simple technique will yield terrible results if the
        # total delta is too large.
        MAX_INTEGRATION_ANGLE = math.radians(30)

        assert abs(roll_rate_rps * dt_s) < MAX_INTEGRATION_ANGLE
        assert abs(pitch_rate_rps * dt_s) < MAX_INTEGRATION_ANGLE
        assert abs(yaw_rate_rps * dt_s) < MAX_INTEGRATION_ANGLE

        return Quaternion(1.0,
                          0.5 * pitch_rate_rps * dt_s,
                          0.5 * roll_rate_rps * dt_s,
                          -0.5 * yaw_rate_rps * dt_s).normalized()
