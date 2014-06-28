#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import argparse
import ConfigParser
import logging
import pygame
import sys
import time

import trollius as asyncio
from trollius import From, Task

import trollius_trace

import leg_ik
import ripple_gait
import servo_controller

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

        while True:
            yield From(self._idle_state())

            # Run the current command as long as one is present.
            yield From(self._run_command())

            # Transition back to the idle state.
            yield From(self._transition_to_idle())

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
                servo_pose, pose_time=3 * UPDATE_TIME))

        while self.next_command is None:
            yield From(self.command_event.wait())

    @asyncio.coroutine
    def _run_command(self):
        self.mode = self.COMMAND
        self.gait.set_state(self.state, ripple_gait.Command())
        
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
            except ripple_gait.NotSupported:
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
        self.gait.set_command(ripple_gait.Command())

        yield From(self._run_while(
                lambda: self.gait.is_command_pending(),
                allow_new=False))

        class PhaseChecker(object):
            def __init__(self, parent):
                self.phase_delta = 0.0
                self.parent = parent
                self.old_phase = self.parent.state.phase

            def check(self):
                self.phase_delta += _wrap_phase(
                    self.parent.state.phase - self.old_phase)
                self.old_phase = self.parent.state.phase
                return self.phase_delta < 0.95

        checker = PhaseChecker(self)

        yield From(self._run_while(checker.check, allow_new=False))

        def any_non_stance():
            for leg in self.state.legs.values():
                if leg.mode != ripple_gait.STANCE:
                    return True
            return False

        yield From(self._run_while(any_non_stance, allow_new=False))
        
        
def _wrap_phase(delta):
    if delta < 0.0:
        return delta + 1.0
    return delta

def load_ik(config):
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

    command = ripple_gait.Command()
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
def start(args):
    config = ConfigParser.ConfigParser()
    config.read(args.config)

    leg_ik_map = load_ik(config)

    ripple_config = ripple_gait.RippleConfig.read_settings(
        config, 'gaitconfig', leg_ik_map)

    gait = ripple_gait.RippleGait(ripple_config)

    servo = yield From(servo_controller.servo_controller(
              args.servo,
              serial_port='/dev/ttyUSB0',
              model_name=args.model))

    driver = GaitDriver(gait, servo)

    driver_task = Task(driver.run())
    input_task = Task(handle_input(driver))

    done, pending = yield From(
        asyncio.wait([driver_task, input_task],
                     return_when=asyncio.FIRST_EXCEPTION))
    for x in done:
        x.result()

def main():
    logging.basicConfig(level=logging.WARN, stream=sys.stdout)

    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument('-c', '--config', help='configuration to use')
    parser.add_argument('-s', '--servo', help='type of servo',
                        default='gazebo')
    parser.add_argument('-m', '--model', help='gazebo model name',
                        default='mj_mech')

    args = parser.parse_args()

    task = Task(start(args))
    asyncio.get_event_loop().run_until_complete(task)

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        if hasattr(e, '__frames__'):
            print ''.join(getattr(e, '__frames__'))
        else:
            raise
