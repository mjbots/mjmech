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

import pylab

class PlotAccumulator(object):
    def __init__(self, filename):
        self.fd = open(filename, 'w')
        self.fd.write('PLOT_ACCUMULATOR\n')
        self.identifiers = {}
        self.next_identifier = 1
        self.time_set = False

    def add(self, **kwargs):
        assert self.time_set

        for key, value in kwargs.iteritems():
            if isinstance(value, dict):
                self.add(**dict((key + '.' + subkey, subvalue)
                                for subkey, subvalue in value.iteritems()))
            else:
                if not key in self.identifiers:
                    self.identifiers[key] = self.next_identifier
                    self.fd.write('I %d %s\n' % (self.next_identifier, key))
                    self.next_identifier += 1

                self.fd.write('V %d %.6g\n' % (self.identifiers[key], value))

    def advance_time(self, time=None):
        self.time_set = True

        self.fd.write('T %.6f\n' % time)

    def close(self):
        self.fd.close()

class NullAccumulator(object):
    def add(self, **kwargs):
        pass

    def advance_time(self, time=None):
        pass

    def close(self):
        pass

class PlotReader(object):
    def __init__(self, filename):
        self.identifiers = {}

        '''Dict mapping names to (time_list, value_list)'''
        self.items = {}
        self.curtime = None

        with open(filename, 'r') as fd:
            for line in fd.readlines():
                line = line.strip()
                fields = line.split(' ')
                if line.startswith('I '):
                    _, identifier, name = fields
                    self.identifiers[int(identifier)] = name
                elif line.startswith('T '):
                    self.curtime = float(fields[1])
                elif line.startswith('V '):
                    _, identifier, value = fields
                    value = float(value)
                    name = self.identifiers[int(identifier)]
                    times, values = self.items.setdefault(name, ([], []))
                    if len(times) == 0 or self.curtime != times[-1]:
                        times.append(self.curtime)
                        values.append(value)
                    elif times[-1] == self.curtime:
                        values[-1] = value


    def list(self):
        return self.items.keys()

    def get(self, name):
        return self.items[name]

    def plot(self, name):
        times, values = self.items[name]
        assert len(times) == len(values)

        MAX_LENGTH = 10000
        if len(times) > MAX_LENGTH:
            subsample = int(len(times) / MAX_LENGTH)
            times = [times[x] for x in xrange(0, len(times), subsample)]
            values = [sum(values[x:x+subsample])/subsample
                      for x in xrange(0, len(values), subsample)]

        pylab.plot(times, values, label=name)
