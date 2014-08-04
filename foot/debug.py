#!/usr/bin/python

import optparse
import serial
import time

def main():
    parser = optparse.OptionParser()
    parser.add_option('-p', '--port', help='serial port')

    options, args = parser.parse_args()

    fd = serial.Serial(port=options.port, baudrate=38400)
    fd.write('\n')
    fd.write('VER\n')
    ver = fd.readline().strip()
    print "Version:", ver

    while True:
        fd.write('I2C s60w12ff\n')
        result = fd.readline()
        assert result.startswith('<I2C OK')
        time.sleep(0.005)
        fd.write('I2C s60w00r04\n')
        result = fd.readline()
        data = result.split(' ')[2]
        pressure = int(data[0:4], 16) >> 6
        temp = int(data[4:8], 16) >> 6
        
        print pressure, temp

        time.sleep(1)

if __name__ == '__main__':
    main()
    
