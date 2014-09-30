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

class TestCase(object):
    def advance(self, dt):
        raise NotImplementedError

    def gyro(self):
        raise NotImplementedError

    def accel_yz_mps2(self):
        raise NotImplementedError

    def expected_pitch(self):
        raise NotImplementedError

class StationaryTest(TestCase):
    def __init__(self, pitch_deg=0.0):
        self.pitch_deg = pitch_deg
        self.y_g = math.sin(math.radians(self.pitch_deg))
        self.z_g = math.cos(math.radians(self.pitch_deg))

    def name(self):
        if self.pitch_deg == 0.0:
            return 'stationary'
        return 'stationary %d' % int(self.pitch_deg)

    def advance(self, dt):
        pass

    def gyro(self):
        return 0.0

    def accel_yz_mps2(self):
        return self.y_g * 9.81, self.z_g * 9.81

    def expected_pitch(self):
        return math.radians(self.pitch_deg)

    def pos(self):
        return 0., 0.
