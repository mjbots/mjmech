#!/usr/bin/env python
# Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.

import capnp
import StringIO
import tempfile
import unittest

import telemetry_log


class TelemetryLogTest(unittest.TestCase):
    def test_header(self):
        output = StringIO.StringIO()
        w = telemetry_log.Writer(output)
        w.close()
        self.assertEquals(output.getvalue(), telemetry_log._HEADER)

    def test_basic(self):
        output = StringIO.StringIO()
        w = telemetry_log.Writer(output)

        schema = '''@0xddf12c7d6250ff51;

struct Sample {
  stuff @0 :Text;
  year @1 :Int16;
}
'''

        record1 = w.register('record1', schema, 'Sample')

        tf = tempfile.NamedTemporaryFile()
        tf.write(schema)
        tf.flush()

        cp = capnp.load(tf.name)

        sample = cp.Sample.new_message()
        sample.stuff = 'hello'
        sample.year = 1975

        record1(sample)

        sample = cp.Sample.new_message()
        sample.year = 1985
        sample.stuff = 'more stuff'

        record1(sample)

        w.close()

        r = telemetry_log.BulkReader(StringIO.StringIO(output.getvalue()))

        self.assertEquals(r.records().keys(), ['record1'])

        everything = r.get()

        self.assertEquals(everything.keys(), ['record1'])
        self.assertEquals(len(everything['record1']), 2)
        self.assertEquals(everything['record1'][0].year, 1975)
        self.assertEquals(everything['record1'][1].year, 1985)


if __name__ == '__main__':
    unittest.main()
