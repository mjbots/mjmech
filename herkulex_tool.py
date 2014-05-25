#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import optparse

import herkulex

def get_address(options):
    if options.address is None:
        return herkulex.HerkuleX.BROADCAST

    return int(options.address)

def do_enumerate(servo, options):
    print servo.enumerate()

def do_set_address(servo, options):
    address = int(options.set_address)
    if address == servo.BROADCAST:
        raise RuntimeError('address cannot be set using broadcast address')
    servo.eep_write(get_address(options), servo.EEP_ID, [ address ])

def do_reboot(servo, options):
    servo.reboot(get_address(options))

def do_status(servo, options):
    print servo.status(get_address(options))

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

    (options, args) = parser.parse_args()

    actions = {
        'enumerate': do_enumerate,
        'set_address': do_set_address,
        'reboot': do_reboot,
        'status': do_status,
        }

    action_func = None
    for key in actions.keys():
        if hasattr(options, key) and getattr(options, key) is not None:
            if action_func is not None:
                raise RuntimeError('more than one action set')
            action_func = actions[key]

    servo = herkulex.HerkuleX(options.device)

    if action_func is not None:
        action_func(servo, options)
        return
    else:
        raise RuntimeError('no action specified')

if __name__ == '__main__':
    main()
