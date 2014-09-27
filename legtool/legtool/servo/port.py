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

'''A transparent mechanism of opening either serial ports or other
transports based on a string selector.'''

import socket
import trollius as asyncio
from trollius import From, Return

from ..async import asyncio_serial

class AsyncSocket(object):
    def __init__(self):
        self.socket = socket.socket()
        self.socket.setblocking(False)
        self.write_lock = asyncio.Lock()
        self.read_lock = asyncio.Lock()

    @asyncio.coroutine
    def connect(self, address):
        loop = asyncio.get_event_loop()
        host, port = address.split(':')

        yield From(loop.sock_connect(self.socket, (host, int(port))))

    @asyncio.coroutine
    def write(self, data):
        loop = asyncio.get_event_loop()
        yield From(loop.sock_sendall(self.socket, data))

    @asyncio.coroutine
    def read(self, size):
        loop = asyncio.get_event_loop()
        result = yield From(loop.sock_recv(self.socket, size))

        raise Result(result)


@asyncio.coroutine
def _open_string(port_name, baud_rate):
    if port_name.startswith('tcp:'):
        _, address = port_name.split(':', 1)

        result = AsyncSocket()
        yield From(result.connect(address))

        raise Return(result)

    raise Return(asyncio_serial.AsyncioSerial(
            port_name, baudrate=baud_rate))


@asyncio.coroutine
def open(port_name, baud_rate=115200):
    if isinstance(port_name, str) or isinstance(port_name, unicode):
        result = yield From(_open_string(port_name, baud_rate))
        raise Return(result)

    # Assume it is a file like object with appropriate lock members.
    raise Return(port_name)
