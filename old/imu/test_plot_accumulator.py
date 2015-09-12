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

import tempfile
import unittest

from plot_accumulator import PlotAccumulator, PlotReader

class PlotAccumulatorTest(unittest.TestCase):
    def test_basic(self):
        with tempfile.NamedTemporaryFile() as fd:
            dut = PlotAccumulator(fd.name)

            dut.advance_time(1.0)
            dut.add(stuff=1.0)
            dut.add(ld={'hi':0.5,'there':1e-5})
            dut.advance_time(2.5)
            dut.add(stuff=2.0)
            dut.add(more=9e9)
            dut.close()

            dut = PlotReader(fd.name)
            self.assertEqual(sorted(dut.list()),
                             sorted(['stuff',
                                     'ld.hi',
                                     'ld.there',
                                     'more']))

            times, values = dut.get('stuff')
            self.assertEqual(times, [1.0, 2.5])
            self.assertEqual(values, [1.0, 2.0])

            times, values = dut.get('ld.hi')
            self.assertEqual(times, [1.0])
            self.assertEqual(values, [0.5])

            times, values = dut.get('more')
            self.assertEqual(times, [2.5])
            self.assertEqual(values, [9e9])


if __name__ == '__main__':
    unittest.main()
