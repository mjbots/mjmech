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


import ConfigParser
import logging
import optparse
import pygame
import sys
import time

import trollius as asyncio
from trollius import From, Task, Return

from legtool.async import trollius_trace
from legtool.servo import selector
from legtool.gait import leg_ik
from legtool.gait import ripple

UPDATE_TIME = 0.04

class GaitDriver(object):
    '''This class takes a gait controller, and a servo controller
    object, and accepts commands to sequence them in time.  It is
    intended to be used as a persistent module controlling the legs on
    a mech.

    Any given command may take some time to take effect, due to the
    need to move all actuators through a feasible transition path.

    It requires that the asyncio event loop's time be monotonic.
    Either by using a monotonic time source, or by requiring the
    system clock to be monotonic.
    '''

    IDLE, COMMAND, IDLING = range(3)
    
    def __init__(self, gait, servo):
        self.gait = gait
        self.servo = servo
        self.state = None
        
        self.mode = self.IDLE
        
        # This event should be set anytime a next_command is present.
        self.command_event = asyncio.Event()
        self.next_command = None

    @asyncio.coroutine
    def run(self):
        '''The run method starts gait driver operation.  Once invoked,
        it will begin commanding the servos.'''

        # It would be nice and/or cool to step the legs up into
        # position.  It may even be required for mechs where
        # instantaneous torque is insufficient to jump all the legs up
        # at once.  However, just forcing us to the gait's idle state
        # is easier for now.

        self.state = self.gait.get_idle_state()
        self.gait.set_state(self.state, ripple.Command())

        do_idle = True

        while True:
            if do_idle:
                yield From(self._idle_state())

            # Run the current command as long as one is present.
            yield From(self._run_command())

            # Transition back to the idle state.
            do_idle = yield From(self._transition_to_idle())

    def set_command(self, command):
        '''Command the mech to begin motion.

        :param command: The current command to begin executing
        :type command: An instance of gait_command.Command
        '''

        self.next_command = command
        self.command_event.set()

    def set_idle(self):
        '''Command the mech to assume an idle state where the legs are
        motionless.'''

        self.next_command = None
        self.command_event.clear()
        
        if self.mode == self.IDLE:
            pass
        elif self.mode == self.COMMAND:
            self.mode = self.IDLING

    @asyncio.coroutine
    def _idle_state(self):
        self.mode = self.IDLE
        self.state = self.gait.get_idle_state()
        servo_pose = self.state.command_dict()

        yield From(self.servo.set_pose(
                servo_pose, pose_time=0.5))

        # TODO jpieper: It would be nice to periodically set the idle
        # command while waiting for something new.
        while self.next_command is None:
            yield From(self.command_event.wait())

    @asyncio.coroutine
    def _run_command(self):
        self.mode = self.COMMAND
        
        yield From(self._run_while(
                lambda: self.mode == self.COMMAND,
                allow_new=True))

    @asyncio.coroutine
    def _run_while(self, condition, allow_new):
        current_time = time.time()
        
        while condition():
            if allow_new and self.next_command:
                self.gait.set_command(self.next_command)
                self.next_command = None
                self.command_event.clear()

            self.state = self.gait.advance_time(UPDATE_TIME)
            try:
                servo_pose = self.state.command_dict()
                yield From(self.servo.set_pose(
                        servo_pose, pose_time=3 * UPDATE_TIME))
            except ripple.NotSupported:
                pass

            delay = (current_time + UPDATE_TIME) - time.time()
            if delay > 0.0:
                yield From(asyncio.sleep(delay))
            current_time += UPDATE_TIME

    @asyncio.coroutine
    def _transition_to_idle(self):
        # Command 0 speed, wait for the command to be active, then
        # wait for at least 1 phase, then wait for all legs to be in
        # stance.
        self.gait.set_command(ripple.Command())

        yield From(self._run_while(
                lambda: (self.gait.is_command_pending() and
                         not self.next_command),
                allow_new=False))

        if self.next_command:
            raise Return(False)

        class PhaseChecker(object):
            def __init__(self, parent):
                self.phase_delta = 0.0
                self.parent = parent
                self.old_phase = self.parent.state.phase

            def check(self):
                self.phase_delta += _wrap_phase(
                    self.parent.state.phase - self.old_phase)
                self.old_phase = self.parent.state.phase
                return (self.phase_delta < 0.95 and
                        not self.parent.next_command)

        checker = PhaseChecker(self)

        yield From(self._run_while(checker.check, allow_new=False))

        if self.next_command:
            raise Return(False)

        def any_non_stance():
            if self.next_command:
                return False

            for leg in self.state.legs.values():
                if leg.mode != ripple.STANCE:
                    return True
            return False

        yield From(self._run_while(any_non_stance, allow_new=False))

        if self.next_command:
            raise Return(False)

        raise Return(True)


class MechDriver(object):
    def __init__(self, options):
        self.options = options
        self.command_time = time.time()

        self.driver = None
        self.servo = None

        config = ConfigParser.ConfigParser()
        config.read(options.config)

        self.leg_ik_map = _load_ik(config)

        self.ripple_config = ripple.RippleConfig.read_settings(
            config, 'gaitconfig', self.leg_ik_map)

        self.gait = ripple.RippleGait(self.ripple_config)

    @asyncio.coroutine
    def run(self):
        kwargs = {}
        if self.options.serial_port:
            kwargs['serial_port'] = self.options.serial_port
        if self.options.model_name:
            kwargs['model_name'] = self.options.model_name
        self.servo = yield From(selector.select_servo(
                self.options.servo,
                **kwargs))

        self.driver = GaitDriver(self.gait, self.servo)

        idle_task = Task(self._make_idle())
        driver_task = Task(self.driver.run())

        done, pending = yield From(
            asyncio.wait([idle_task, driver_task],
                         return_when=asyncio.FIRST_EXCEPTION))

        for x in done:
            x.result()

    def set_command(self, command):
        self.command_time = time.time()
        self.driver.set_command(command)

    def set_idle(self):
        self.command_time = time.time()
        self.driver.set_idle()

    def set_turret(self, pan_deg, tilt_deg):
        if not self.servo:
            return

        Task(self.servo.set_pose({12: pan_deg, 13: tilt_deg}))

    @asyncio.coroutine
    def _make_idle(self):
        while True:
            now = time.time()
            if now - self.command_time > 2.0:
                self.driver.set_idle()
            yield From(asyncio.sleep(0.5))

    @staticmethod
    def add_options(parser):
        parser.add_option('-c', '--config', help='configuration to use')
        parser.add_option('-s', '--servo', help='type of servo',
                          default='gazebo')
        parser.add_option('-m', '--model-name', help='gazebo model name',
                          default='mj_mech')
        parser.add_option('-p', '--serial-port', help='serial port to use',
                          default='/dev/ttyUSB0')
        
        
def _wrap_phase(delta):
    if delta < 0.0:
        return delta + 1.0
    return delta

def _load_ik(config):
    leg_ik_map = {}
    for section in config.sections():
        if section.startswith('ikconfig.leg.'):
            leg_num = int(section.split('.')[-1])
            ikconfig = leg_ik.Configuration.read_settings(
                config, section)
            leg_ik_map[leg_num] = leg_ik.LizardIk(ikconfig)

    return leg_ik_map

@asyncio.coroutine
def handle_input(driver):
    pygame.init()
    pygame.joystick.init()
    clock = pygame.time.Clock()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print "name=", joystick.get_name()
    print "numaxes=", joystick.get_numaxes()
    print "numbuttons=", joystick.get_numbuttons()

    command = ripple.Command()
    idle = True
        
    while True:
        pygame.event.pump()
        
        x_speed = joystick.get_axis(0)
        if abs(x_speed) < 0.1:
            x_speed = 0.0
        y_speed = -joystick.get_axis(1)
        if abs(y_speed) < 0.1:
            y_speed = 0.0
        rot_speed = joystick.get_axis(3)
        if abs(rot_speed) < 0.1:
            rot_speed = 0.0

        print x_speed, y_speed, rot_speed

        command.translate_x_mm_s = x_speed * 100
        command.translate_y_mm_s = y_speed * 250
        command.rotate_deg_s = rot_speed * 30

        if x_speed != 0.0 or y_speed != 0.0 or rot_speed != 0.0:
            driver.set_command(command)
            idle = False
        else:
            if not idle:
                driver.set_idle()
                idle = True
        
        yield From(asyncio.sleep(0.05))
        clock.tick(10)

@asyncio.coroutine
def start(options):
    gait_driver = MechDriver(options)
    driver_task = Task(gait_driver.run())
    input_task = Task(handle_input(gait_driver))

    done, pending = yield From(
        asyncio.wait([driver_task, input_task],
                     return_when=asyncio.FIRST_EXCEPTION))
    for x in done:
        x.result()

def main():
    logging.basicConfig(level=logging.WARN, stream=sys.stdout)

    parser = optparse.OptionParser(description=__doc__)

    MechDriver.add_options(parser)

    options, args = parser.parse_args()

    task = Task(start(options))
    asyncio.get_event_loop().run_until_complete(task)

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        if hasattr(e, '__frames__'):
            print ''.join(getattr(e, '__frames__'))
        else:
            raise
