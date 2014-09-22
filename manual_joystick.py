#!/usr/bin/python
# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import trollius as asyncio
from trollius import Task, From

import joystick
import linux_input

@asyncio.coroutine
def run(joy):
    while True:
        ev = yield From(joy.read())

        if ev.ev_type != linux_input.EV.ABS:
            continue

        print ev, joy.absinfo(linux_input.ABS(ev.code)).scaled()

def main():
    enumerator = joystick.JoystickEnumerator(lambda x: 1)
    if len(enumerator.joysticks()) == 0:
        raise RuntimeError('no joysticks found')

    joy = enumerator.joysticks()[0]
    print joy
    axes = joy.get_features(linux_input.EV.ABS)
    for axis in axes:
        print axis, joy.absinfo(axis)

    task = Task(run(joy))
    asyncio.get_event_loop().run_until_complete(task)


if __name__ == '__main__':
    main()
