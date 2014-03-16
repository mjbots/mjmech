#!/usr/bin/python
"""This modules controls 2 rc servoes via Firmata firmware (comes stock with arduino)
"""
import os
import sys
import serial
import time

BAUDRATE = 57600
PORT = "/dev/ttyACM99"
PIN_X = 3
PIN_Y = 5

MODE_SERVO = 4
SYSTEM_RESET = 0xFF
START_SYSEX = 0xF0 # 240
END_SYSEX = 0xF7 
REPORT_FIRMWARE = 0x79

VERBOSE = 0
def write_bin(dev, blist):
    if VERBOSE:
        print "TX:", "-".join("%02X" % b for b in blist)
    dev.write("".join(chr(b) for b in blist))

def read_bin(dev, count, null_ok=False):
    msg = dev.read(count)
    blist = [ord(m) & 0xFF for m in msg]
    if VERBOSE:
        print "RX:", "-".join("%02X" % b for b in blist)
    if not (null_ok or len(blist)):
        raise Exception('serial port timed out')
    return blist

def set_servo(dev, pin, value, header=False):
    """Generate set servo command. Arg is 0..180"""
    pkt = []
    if header:
        pkt += [0xF4, pin, MODE_SERVO]
    pkt += [0xE0 | pin, value & 0x7F, (value >> 7) & 0x7F]
    write_bin(dev, pkt)

def main():
    dev = serial.Serial(PORT, baudrate=BAUDRATE)
    dev.setTimeout(2)

    # request firmware version (as a way to make sure board is connected)
    write_bin(dev, [SYSTEM_RESET])
    write_bin(dev, [START_SYSEX, REPORT_FIRMWARE, END_SYSEX])
    fwinfo = read_bin(dev, 5)
    assert fwinfo[:2] == [START_SYSEX, REPORT_FIRMWARE], fwinfo
    while fwinfo[-1] != END_SYSEX:
        fwinfo += read_bin(dev, 1)
    # Lazy decoding of a name -- assuming it will be 8 bit ASCII only.
    print 'Found Firmata ver=%d.%d name=%r' % (
        fwinfo[2], fwinfo[3], ''.join(chr(b) for b in fwinfo[4:-1] if b))

    seqvals = range(90, 120) + range(120, 60, -1) + range(60, 90+1)
    set_servo(dev, PIN_X, seqvals[0], True)
    set_servo(dev, PIN_Y, seqvals[0], True)
    
    for cnt in range(3):
        for v in seqvals:
            set_servo(dev, PIN_X, v)
            time.sleep(0.1)
        for v in seqvals:
            set_servo(dev, PIN_Y, v)
            time.sleep(0.1)
    

if __name__ == '__main__':
    main()
