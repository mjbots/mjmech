#!/usr/bin/python

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

'''A simple command line tool for commanding and monitoring Dongbu
HerkuleX servos.'''

import trollius as asyncio
from trollius import From, Return
import optparse
import sys
import time

from legtool.servo import herkulex

def get_address(options):
    if options.address is None:
        return herkulex.HerkuleX.BROADCAST

    return int(options.address)

@asyncio.coroutine
def do_enumerate(servo, options):
    print(yield From(servo.enumerate()))

@asyncio.coroutine
def do_set_address(servo, options):
    address = int(options.set_address)
    if address == servo.BROADCAST:
        raise RuntimeError('address cannot be set using broadcast address')
    yield From(
        servo.eep_write(get_address(options), servo.EEP_ID, [ address ]))

@asyncio.coroutine
def do_reboot(servo, options):
    yield From(servo.reboot(get_address(options)))

@asyncio.coroutine
def do_status(servo, options, do_print=True):
    addr = get_address(options)
    #status = yield From(servo.status(addr))
    position, status = yield From(servo.position(addr, return_status=True))
    result = 'Servo %d: (0x%02x, 0x%02x) %s | pos %d' % (
        addr, status.reg48, status.reg49,
        ','.join(status.active_flags()) or 'none',
        position or -1)
    if do_print:
        print result
    else:
        raise Return(result)

@asyncio.coroutine
def do_status_loop(servo, options):
    t0 = time.time()
    last_status = None
    while True:
        dt = time.time() - t0
        status = yield From(do_status(servo, options, do_print=False))
        if status != last_status:
            print '[%6.2f]' % dt, status
            last_status = status
        yield From(asyncio.sleep(0.1))

@asyncio.coroutine
def do_voltage(servo, options):
    print(yield From(servo.voltage(get_address(options))))

@asyncio.coroutine
def do_temperature(servo, options):
    print(yield From(servo.temperature_C(get_address(options))))

@asyncio.coroutine
def do_write_ram(servo, options, addr, data):
    yield From(servo.ram_write(get_address(options), addr, data))

@asyncio.coroutine
def do_blink_led(servo, options):
    addr = get_address(options)
    try:
        for _ in xrange(10):
            for leds in [servo.LED_RED, servo.LED_GREEN, servo.LED_BLUE, 0]:
                yield From(servo.set_leds(addr, leds))
                yield From(asyncio.sleep(0.2))
                sys.stderr.write(str(leds))
                sys.stderr.flush()
    finally:
        yield From(servo.set_leds(addr, 0))
        print >>sys.stderr, '0'

@asyncio.coroutine
def do_read_config(servo, options):
    result = yield From(servo.read_servo_config(get_address(options)))
    for k, v in sorted(result.items()):
        print " %-14s: %r" % (k, v)

def main():
    parser = optparse.OptionParser()
    parser.add_option('-a', '--address', default=None,
                      help='servo to communicate with')
    parser.add_option('-d', '--device', default='/dev/ttyUSB0',
                      help='serial port device')
    parser.add_option('-e', '--enumerate', action='store_true', default=None,
                      help='enumerate servos on the bus')
    parser.add_option('--set-address', dest='set_address', default=None,
                      help='set all servos on the bus to the given address')
    parser.add_option('-r', '--reboot', action='store_true', default=None,
                      help='reboot servos')
    parser.add_option('-s', '--status', action='store_true', default=None,
                      help='query status of servo')
    parser.add_option('-S', '--status-loop', action='store_true', default=None,
                      help='continuously query status of servo and print '
                      'any changes')
    parser.add_option('-v', '--voltage', action='store_true', default=None,
                      help='query voltage of servo')
    parser.add_option('-t', '--temperature', action='store_true', default=None,
                      help='query temperature of servo')
    parser.add_option('-b', '--blink_led', action='store_true', default=None,
                      help='Blink servo\'s LEDs')
    parser.add_option('-w', '--write-ram', action='append', default=[],
                      metavar='ADDR:D0:[D1..]',
                      help="Execute 'write ram' command. May be specified "
                      "multiple times")
    parser.add_option('-c', '--read-config', action='store_true', default=None,
                      help='Read and display servo config')

    (options, args) = parser.parse_args()

    if args:
        parser.error('No arguments accepted')

    actions = {
        'enumerate': do_enumerate,
        'set_address': do_set_address,
        'reboot': do_reboot,
        'status': do_status,
        'status_loop': do_status_loop,
        'voltage': do_voltage,
        'temperature': do_temperature,
        'blink_led': do_blink_led,
        'read_config': do_read_config,
        }


    write_commands = []
    for cmd in options.write_ram:
        write_commands.append(
            [int(p, 0) for p in cmd.split(':')])

    servo = herkulex.HerkuleX(options.device)

    @asyncio.coroutine
    def run_actions():
        yield From(servo.start())

        ran_any = False
        for cmd in write_commands:
            yield From(do_write_ram(servo, options, cmd[0], cmd[1:]))
            ran_any = True

        for key, callback in sorted(actions.items()):
            if getattr(options, key) is not None:
                yield From(callback(servo, options))
                ran_any = True

        if not ran_any:
            raise RuntimeError('no action specified')

    loop = asyncio.get_event_loop()
    loop.run_until_complete(run_actions())

if __name__ == '__main__':
    main()
