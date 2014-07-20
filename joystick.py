# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import glob

import linux_input

class JoystickEnumerator(object):
    '''The JoystickEnumerator scans all input devices in the system
    looking for input devices which might be useful.

    metric is a callable, which when passed an InputDevice returns an
    integer score measuring how good of an input device it is.
    Negative values mean that the device is unusable.  Positive values
    indicate how good it is.
    '''
    def __init__(self, metric):
        self._joysticks = []
        
        for path in glob.glob('/dev/input/event*'):
            device = None
            
            try:
                device = linux_input.InputDevice(path)
            except Exception as e:
                continue

            score = metric(device)
            self._joysticks.append((score, device))

        self._joysticks.sort(key=lambda x: -x[0])
    

    def joysticks(self):
        '''Return a list of joystick devices, sorted such that the
        best matches are first.'''

        return [x[1] for x in self._joysticks]
