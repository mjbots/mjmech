#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import eventlet
import serial
import struct
import time

LOG_WRITE_START = 1
LOG_WRITE_PARTIAL = 2
LOG_WRITE_DONE = 3

LOG_READ_START = 17
LOG_READ_PARTIAL = 18
LOG_READ_DONE = 19

class EventletSerial(object):
    def __init__(self, *args, **kwargs):
        self.log = None
        if 'logfile' in kwargs:
            self.logfile = kwargs.pop('logfile')
            if self.logfile is not None:
                self.log = open(self.logfile, 'w')

        kwargs['timeout'] = 0
        self.raw_serial = serial.Serial(*args, **kwargs)
        self.sem_write = eventlet.semaphore.Semaphore()
        self.sem_read = eventlet.semaphore.Semaphore()

    def write(self, data):
        assert self.sem_write.locked()

        self.emit_log(LOG_WRITE_START, data)

        to_write = data
        while len(to_write):
            eventlet.hubs.trampoline(self.raw_serial.fileno(), write=True)
            written = self.raw_serial.write(to_write)
            self.emit_log(LOG_WRITE_PARTIAL, struct.pack('>l', written))
            to_write = to_write[written:]

        self.emit_log(LOG_WRITE_DONE, '')

    def read(self, size):
        assert self.sem_read.locked()

        self.emit_log(LOG_READ_START, struct.pack('>l', size))

        bytes_remaining = size
        read_so_far = ""
        while bytes_remaining > 0:
            eventlet.hubs.trampoline(self.raw_serial.fileno(), read=True)
            this_read = self.raw_serial.read(bytes_remaining)
            self.emit_log(LOG_READ_PARTIAL, this_read)
            bytes_remaining -= len(this_read)
            read_so_far += this_read

        self.emit_log(LOG_READ_DONE, '')

        return read_so_far

    def readline(self):
        result = ''
        while True:
            data = self.read(1)
            result += data
            if data == '\r' or data == '\n':
                return result

    def emit_log(self, log_key, data):
        if self.log is None:
            return

        self.log.write(
            struct.pack('>dll', time.time(), log_key, len(data)) + data)
