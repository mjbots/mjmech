#!/usr/bin/python

# Copyright 2012-2014 Josh Pieper, jjp@pobox.com.  All rights reserved.
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
import random

class ImuData(object):
    pitch_rate = 0.0
    yaw_rate = 0.0
    roll_rate = 0.0

class SensorErrorModel(object):
    '''This models the error associated with a single rate or
    acceleration sensor.  The error consists of multiple components,
    some of which evolve over time, and some may be non-linearly
    related to the sensed output.

    It does not currently model gyro g-sensitivity, or cross axis
    coupling.
    '''
    def __init__(self):
        # For Pololu Mini-IMU v1
        self.angle_random_walk = 4.629e-4 # rad/sqrt(s)
        self.bias_stability = 6.076e-5 # rad/s
        self.rate_random_walk = 4.03e-6 # rad/sqrt(s)

        self.rng = random.Random()
        self.roll_rate, self.pitch_rate, self.yaw_rate = [
            InertialErrorModel(
                self.rng, 100.0,
                self.angle_random_walk,
                self.bias_stability,
                self.rate_random_walk)
            for x in range(3)
            ]

    def sample(self, actual):
        result = ImuData()

        result.pitch_rate = actual.pitch_rate + self.pitch_rate.sample()
        result.roll_rate = actual.roll_rate + self.roll_rate.sample()
        result.yaw_rate = actual.yaw_rate + self.yaw_rate.sample()

        return result

class InertialErrorModel(object):
    def __init__(self,
                 rng,
                 sample_frequency_hz,
                 angle_random_walk_rad_sqrt_s,
                 bias_stability_rad_s,
                 rate_random_walk_rad_sqrt_s,
                 random_initial_bias=True):
        bw_sqrt_hz = math.sqrt(sample_frequency_hz)
        self._white_noise = WhiteNoise(
            rng, bw_sqrt_hz * angle_random_walk_rad_sqrt_s)

        bias_stddev_factor = math.sqrt(2 * math.log(2) / math.pi)
        bias_raw = PowerLawNoise(
            rng, 1, bias_stability_rad_s / bias_stddev_factor)
        bias_sampled = ResampleFilter(bias_raw, 0.5, sample_frequency_hz)
        self._bias = ChebyshevFilter(
            bias_sampled,
            sample_frequency_hz, 0.2,
            ChebyshevFilter.LOW_PASS,
            0, 2)

        self._rate_walk = BrownianMotion(
            rng, rate_random_walk_rad_sqrt_s / bw_sqrt_hz)

        if random_initial_bias:
            self._overall_bias = rng.gauss(
                0, bw_sqrt_hz * angle_random_walk_rad_sqrt_s)
        else:
            self._overall_bias = 0.0

        self._last_bias = self._overall_bias

    def bias(self):
        return self._last_bias

    def sample(self):
        self._last_bias = self._bias.sample() + self._overall_bias
        return (self._white_noise.sample() +
                self._last_bias +
                self._rate_walk.sample())

class WhiteNoise(object):
    def __init__(self, rng, stddev):
        self._rng = rng
        self._stddev = stddev

    def sample(self):
        return self._rng.gauss(0, self._stddev)

class BrownianMotion(object):
    def __init__(self, rng, stddev):
        self._white_noise = WhiteNoise(rng, stddev)
        self._total = 0.0

    def sample(self):
        self._total += self._white_noise.sample()
        return self._total

class PowerLawNoise(object):
    def __init__(self, rng, alpha, stddev):
        self._alpha = alpha
        self._white_noise = WhiteNoise(rng, stddev)
        self._samples = []

    def sample(self):
        result = self._white_noise.sample()
        a = 1.0

        for i in range(len(self._samples)):
            a = (i - self._alpha / 2.0) * (a / (i + 1))
            result -= a * self._samples[-i - 1]

        self._samples.append(result)
        return result

class ResampleFilter(object):
    def __init__(self, source, source_frequency_hz, dest_frequency_hz):
        self._source = source
        self._ratio = dest_frequency_hz / source_frequency_hz
        self._last_value = 0.0
        self._position = self._ratio

    def sample(self):
        while self._position >= self._ratio:
            self._last_value = self._source.sample()
            self._position -= self._ratio

        self._position += 1.0
        return self._last_value

class CircularQueue(object):
    def __init__(self):
        self._next_write = 0
        self.size = 0
        self._data = []

    def reserve(self, size):
        self._data = [None] * size
        self._next_write = 0
        self.size = 0

    def prepend(self, value):
        self._data[self._next_write] = value
        self._next_write = (self._next_write + 1) % len(self._data)
        self.size = min(self.size + 1, len(self._data))

    def __getitem__(self, i):
        assert i < self.size
        offset = (self._next_write + len(self._data) - 1 - i) % len(self._data)
        return self._data[offset]

class ChebyshevFilter(object):
    LOW_PASS = 0
    HIGH_PASS = 1

    def __init__(self, source, source_frequency, filter_frequency,
                 filter_type, percent_ripple, number_of_poles):
        self._source = source
        assert number_of_poles >= 2 and (number_of_poles % 2) == 0
        assert filter_frequency < 0.5 * source_frequency
        assert percent_ripple >= 0

        result = self._calculate_coefficients(
            filter_frequency / source_frequency,
            filter_type,
            percent_ripple,
            number_of_poles)

        if (sum([abs(x) for x in result.a]) <
            sum([abs(x) for x in result.b]) * 1e-10):
            raise RuntimeError('Computed Chebyshev filter is likely unstable')

        self._a = result.a
        self._b = result.b

        self._input = CircularQueue()
        self._input.reserve(len(self._a))

        self._output = CircularQueue()
        self._output.reserve(len(self._b))

    def sample(self):
        result = 0.0

        self._input.prepend(self._source.sample())
        for i in range(self._input.size):
            result += self._a[i] * self._input[i]

        for i in range(self._output.size):
            result += self._b[i] * self._output[i]

        self._output.prepend(result)
        return result

    @staticmethod
    def _calculate_stage(pole, f_c, filter_type,
                         percent_ripple, number_of_poles):
        rp = -math.cos(math.pi / (number_of_poles * 2) +
                       (pole - 1) * (math.pi / number_of_poles))
        ip = math.sin(math.pi / (number_of_poles * 2) +
                      (pole - 1) * (math.pi / number_of_poles))

        if percent_ripple != 0.0:
            # Warp from a circle to an ellipse.

            es = math.sqrt((100.0 / (100.0 - percent_ripple)) ** 2 - 1)
            vx = (1.0 / number_of_poles) * math.log(
                (1.0 / es) + math.sqrt((1.0 / (es**2)) + 1))
            kx = (1.0 / number_of_poles) * math.log(
                (1.0 / es) + math.sqrt((1.0 / (es**2)) - 1))
            kx = (math.exp(kx) + math.exp(-kx)) / 2.0
            rp = rp * ((math.exp(vx) - math.exp(-vx)) / 2) / kx
            ip = ip * ((math.exp(vx) + math.exp(-vx)) / 2 ) / kx

        # S-domain to z-domain conversion
        t = 2 * math.tan(0.5)
        w = 2 * math.pi * f_c
        m = rp ** 2 + ip ** 2
        d = 4.0 - 4.0 * rp * t + m * t * t
        x0 = t * t / d
        x1 = 2 * t * t / d
        x2 = t * t / d
        y1 = (8 - 2 * m * t * t) / d
        y2 = (-4 -4 * rp * t - m * t * t) / d

        # LP to LP or LP to HP transform
        if filter_type == ChebyshevFilter.HIGH_PASS:
            k = -math.cos(w / 2 + 0.5) / math.cos(w / 2 - 0.5)
        else:
            k = math.sin(0.5 - w / 2) / math.sin(0.5 + w / 2)

        d = 1 + y1 * k - y2 * k * k

        class Result(object):
            a = [ (x0 - x1 * k + x2 * k * k) / d,
                  (-2 * x0 * k + x1 + x1 * k * k - 2 * x2 * k) / d,
                  (x0 * k * k - x1 * k + x2) / d ]
            b = [ 1,
                  (2 * k + y1 + y1 * k * k - 2 * y2 * k) / d,
                  (-(k * k) - y1 * k + y2) / d ]

        result = Result()
        if filter_type == ChebyshevFilter.HIGH_PASS:
            result.a[1] = -result.a[1]
            result.b[1] = -result.b[1]

        return result

    @staticmethod
    def _calculate_coefficients(f_c, filter_type, percent_ripple,
                                number_of_poles):
        a = [ 0.0 ] * (number_of_poles + 4)
        b = [ 0.0 ] * (number_of_poles + 4)

        a[2] = 1.0
        b[2] = 1.0

        for pole in range(1, number_of_poles / 2 + 1):
            result = ChebyshevFilter._calculate_stage(
                pole, f_c, filter_type, percent_ripple, number_of_poles)

            ta = a[:]
            tb = b[:]

            for i in range(2, len(a)):
                a[i] = (result.a[0] * ta[i] +
                        result.a[1] * ta[i - 1] +
                        result.a[2] * ta[i - 2])
                b[i] = (tb[i] -
                        result.b[1] * tb[i - 1] -
                        result.b[2] * tb[i - 2])

        # Finish combining coefficients
        b[2] = 0
        for i in range(0, len(a) - 2):
            a[i] = a[i + 2]
            b[i] = -b[i + 2]

        # Normalize the gain.
        sa = 0
        sb = 0
        for i in range(0, len(a) - 2):
            if filter_type == ChebyshevFilter.LOW_PASS:
                sa = sa + a[i]
                sb = sb + b[i]
            else:
                sa = sa + a[i] * (-1 ** i)
                sb = sb + b[i] * (-1 ** i)

        gain = sa / (1.0 - sb)

        a = a[0:number_of_poles + 1]
        a = [ x / gain for x in a ]
        b = b[1:number_of_poles + 1]

        class Result(object):
            def __init__(self, a, b):
                self.a = a
                self.b = b

        return Result(a, b)
