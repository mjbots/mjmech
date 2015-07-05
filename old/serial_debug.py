#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import struct
import sys

import eventlet_serial

CODE_MAP = {
    eventlet_serial.LOG_WRITE_START : 'LOG_WRITE_START',
    eventlet_serial.LOG_WRITE_PARTIAL : 'LOG_WRITE_PARTIAL',
    eventlet_serial.LOG_WRITE_DONE : 'LOG_WRITE_DONE',
    eventlet_serial.LOG_READ_START : 'LOG_READ_START',
    eventlet_serial.LOG_READ_PARTIAL : 'LOG_READ_PARTIAL',
    eventlet_serial.LOG_READ_DONE : 'LOG_READ_DONE',
    }

def escape(data):
    return data.replace('\n', '\\n').replace('\r', '\\r')

def format_data(code, data):
    if code == eventlet_serial.LOG_WRITE_START:
        return "'" + escape(data) + "'"
    elif code == eventlet_serial.LOG_WRITE_PARTIAL:
        return struct.unpack('>l', data)[0]
    elif code == eventlet_serial.LOG_WRITE_DONE:
        return ''
    elif code == eventlet_serial.LOG_READ_START:
        return struct.unpack('>l', data)[0]
    elif code == eventlet_serial.LOG_READ_PARTIAL:
        return "'" + escape(data) + "'"
    elif code == eventlet_serial.LOG_READ_DONE:
        return ''

def main():
    log = open(sys.argv[1], 'rb')

    header = struct.Struct('>dll')
    while True:
        header_data = log.read(header.size)
        time, code, length = header.unpack(header_data)
        data = log.read(length)

        print '%f,%s,%s' % (time, CODE_MAP[code], format_data(code, data))

if __name__ == '__main__':
    main()
