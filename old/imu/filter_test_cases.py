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

class RateExceededTest(TestCase):
    '''starts out stationary, runs a large sine wave rate for some
    amount of time, then ends at a new orientation.'''
    def __init__(self, magnitude_dps=1000.0, target_pitch_deg=40.0,
                 duration=100.0, period=5.0):
        self.pitch_deg = 0.0
        self.magnitude_dps = magnitude_dps
        self.target_pitch_deg = target_pitch_deg
        self.duration = duration
        self.period = period
        self.start = 100.0
        self.time = 0.0
        self.rate_dps = 0.0

    def advance(self, dt):
        self.time += dt
        force_pitch = False

        if self.time < self.start:
            self.rate_dps = 0.0
        elif self.time < (self.start + self.duration):
            delta = self.time - self.start
            self.rate_dps = self.magnitude_dps * math.sin(
                2 * math.pi * delta / self.period)
        else:
            delta = self.pitch_deg - self.target_pitch_deg
            max_rate = min(self.magnitude_dps, abs(delta) / dt)
            if max_rate < self.magnitude_dps:
                force_pitch = True
            self.rate_dps = max_rate * (-1 if delta > 0 else 1)

        self.pitch_deg += dt * self.rate_dps
        if force_pitch:
            self.pitch_deg = self.target_pitch_deg

    def gyro(self):
        return math.radians(self.rate_dps)

    def name(self):
        return 'rate_exceed dps=%d' % self.magnitude_dps

    def accel_yz_mps2(self):
        y_g = math.sin(math.radians(self.pitch_deg))
        z_g = math.cos(math.radians(self.pitch_deg))
        return y_g * 9.81, z_g * 9.81

    def expected_pitch(self):
        return math.radians(self.pitch_deg)

    def pos(self):
        return 0., 0.
