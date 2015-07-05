# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import eventlet

def parsehex(data):
    result = ''
    while len(data) >= 2:
        result += chr(int(data[0:2], 16))
        data = data[2:]

    return result

class AvrSerial(object):
    def __init__(self, stream):
        self.stream = stream
        self.queue = eventlet.queue.Queue(0)
        eventlet.spawn(self._read_loop)
        self.sem_write = eventlet.semaphore.Semaphore()
        self.sem_read = eventlet.semaphore.Semaphore()

    def write(self, data):
        with self.stream.sem_write:
            self.stream.write(
                'SRT %s\n' % ''.join(['%02X' % ord(x) for x in data]))

    def read(self, size):
        result = ''
        while len(result) < size:
            result += self.queue.get(block=True)
        return result

    def readline(self):
        result = ''
        while True:
            data = self.read(1)
            result += data
            if data == '\r' or data == '\n':
                return result

    def _read_loop(self):
        with self.stream.sem_read:
            while True:
                line = self.stream.readline()
                if line.startswith('!SRD'):
                    hexdata = line[5:]
                    data = parsehex(hexdata)
                    for x in data:
                        self.queue.put(x)
