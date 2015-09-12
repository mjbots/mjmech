#!/usr/bin/env python
import os
import serial
import sys
import optparse
import time
import re

def main():
    parser = optparse.OptionParser(
        usage='\n'
        '   %prog [opts] /dev/ttyUSB1')
    parser.add_option('-d', '--delay', type='float', default=0.1,
                      help='Interval between commands (seconds, def %default)')
    parser.add_option('-b', '--baudrate', type='int', default=57600,
                      help='Baudrate (default %default)')
    parser.add_option('-W', '--no-wiggle', action='store_true',
                      help='Do not wiggle RTS/DST pins')

    opts, args = parser.parse_args()
    if len(args) != 1:
        parser.error('Exacly 1 arguments required')

    print 'Opening %s at %d' % (args[0], opts.baudrate)
    dev = serial.serial_for_url(args[0], opts.baudrate)
    dev.setTimeout(opts.delay)
    print

    print 'Device opened. Reset MCU now!'
    rxbuff = ''
    sent_once = False
    step = 0
    while True:
        chunk = dev.read(1)
        rxbuff += chunk

        if rxbuff == '':
            # idle -- send command
            if not sent_once:
                print 'Sending command'
                sent_once = True
            dev.write('0 ')
            dev.flush()

            step += 1
            if (step % 5) == 0 and not opts.no_wiggle:
                for n in range(3 + (step & 1)):
                  dev.setDTR(n & 1)
                  dev.setRTS(n & 1)

        elif len(rxbuff) > 64 or (chunk in ['\r', '\n', '']):
            if sent_once and '\x14\x10' in rxbuff:
                break
            # print ignored data
            print '(ignored): %r' % rxbuff
            rxbuff = ''

    if rxbuff != '\x14\x10':
        print 'Got extra data with response: %r' % rxbuff
    print 'Done, in bootloader now.'

if __name__ == '__main__':
    main()
