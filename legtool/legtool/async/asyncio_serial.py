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

'''A simple wrapper for serial port access using python3
asyncio/trollius.'''

import trollius as asyncio
from trollius import From, Return
import serial

class AsyncioSerial(object):
    def __init__(self, *args, **kwargs):
        kwargs['timeout'] = 0
        self.raw_serial = serial.serial_for_url(*args, **kwargs)
        self.write_lock = asyncio.Lock()
        self.read_lock = asyncio.Lock()

    @asyncio.coroutine
    def write(self, data):
        assert self.write_lock.locked()

        loop = asyncio.get_event_loop()

        event = asyncio.Event()
        loop.add_writer(self.raw_serial.fileno(), event.set)
        try:
            while len(data):
                yield From(event.wait())
                event.clear()

                written = self.raw_serial.write(data)
                data = data[written:]
        finally:
            loop.remove_writer(self.raw_serial.fileno())

        raise Return()

    @asyncio.coroutine
    def read(self, size):
        assert self.read_lock.locked()

        loop = asyncio.get_event_loop()
        result = ''
        event = asyncio.Event()
        loop.add_reader(self.raw_serial.fileno(), event.set)
        try:
            while len(result) < size:
                yield From(event.wait())
                event.clear()

                to_read = size - len(result)
                this_read = self.raw_serial.read(to_read)
                result += this_read
        finally:
            loop.remove_reader(self.raw_serial.fileno())

        raise Return(result)

    @asyncio.coroutine
    def readline(self):
        result = ''
        while True:
            data = yield From(self.read(1))
            result += data
            if data == '\r' or data == '\n':
                raise Return(result)
