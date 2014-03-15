#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import eventlet
import serial

class EventletSerial(object):
    def __init__(self, *args, **kwargs):
        kwargs['timeout'] = 0
        self.raw_serial = serial.Serial(*args, **kwargs)
        self.sem = eventlet.semaphore.Semaphore()

    def write(self, data):
        assert self.sem.locked()
        to_write = data
        while len(to_write):
            eventlet.hubs.trampoline(self.raw_serial.fileno(), write=True)
            written = self.raw_serial.write(to_write)
            to_write = to_write[written:]

    def read(self, size):
        assert self.sem.locked()
        bytes_remaining = size
        read_so_far = ""
        while bytes_remaining > 0:
            eventlet.hubs.trampoline(self.raw_serial.fileno(), read=True)
            this_read = self.raw_serial.read(bytes_remaining)
            bytes_remaining -= len(this_read)
            read_so_far += this_read

        return read_so_far
