#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import eventlet
import os
import pygame
import sys

import ConfigParser

import eventlet_serial
import servo_controller

class ServoConfig(object):
    def __init__(self, config):
        self.servo_type = 'herkulex'
        self.servo_port = '/dev/ttyUSB0'
        self.servo_pan_id = 2
        self.servo_tilt_id = 1
        self.servo_pan_sign = -1
        self.servo_tilt_sign = 1

        if not config.has_section('servo'):
            return

        self.servo_type = config.get('servo', 'type', self.servo_type)
        self.servo_port = config.get('servo', 'port', self.servo_port)
        self.servo_pan_id = config.get('servo', 'pan_id', self.servo_pan_id)
        self.servo_tilt_id = config.get('servo', 'tilt_id',
                                        self.servo_tilt_id)
        self.servo_pan_sign = config.get('servo', 'pan_sign',
                                         self.servo_pan_sign)
        self.servo_tilt_sign = config.get('servo', 'tilt_sign',
                                          self.servo_tilt_sign)

class MechTurret(object):
    CONFIG_FILE = os.path.expanduser('~/.config/mturret/mturret.ini')

    def __init__(self):
        config = ConfigParser.ConfigParser()
        config.read(self.CONFIG_FILE)
        self.servo_config = ServoConfig(config)

        self.controller = servo_controller.servo_controller(
            self.servo_config.servo_type,
            serial_port=self.servo_config.servo_port)

        self.avr = eventlet_serial.EventletSerial(
            port='/dev/ttyACM0', baudrate=115200)

        self.controller.enable_power(servo_controller.POWER_ENABLE)

        self._current_laser = 0
        self.toggle_laser()


    def set_orientation(self, yaw_deg, pitch_deg):
        self.controller.set_pose(
            {
                self.servo_config.servo_pan_id :
                    yaw_deg * self.servo_config.servo_pan_sign,
                self.servo_config.servo_tilt_id :
                    pitch_deg * self.servo_config.servo_tilt_sign,
                },
            pose_time=0.2)

    def _toggle_laser(self):
        with self.avr.sem:
            self.avr.write('GPC D 4 %d\n' % self._current_laser)
            self._current_laser = self._current_laser ^ 1

    def toggle_laser(self):
        eventlet.spawn(self._toggle_laser)

    def _move_agitator(self):
        with self.avr.sem:
            self.avr.write('PWM 0 100 200\n')

    def move_agitator(self):
        eventlet.spawn(self._move_agitator)

    def _fire(self):
        with self.avr.sem:
            self.avr.write('PWM 1 200 100\n')

    def fire(self):
        eventlet.spawn(self._fire)

def main():
    turret = MechTurret()

    pygame.init()
    pygame.joystick.init()

    clock = pygame.time.Clock()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print "name=", joystick.get_name()
    print "numaxes=", joystick.get_numaxes()
    print "numbuttons=", joystick.get_numbuttons()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:
                    turret.toggle_laser()
                elif event.button == 1:
                    turret.move_agitator()
                elif event.button == 2:
                    turret.fire()

        pan = joystick.get_axis(0) * 20
        tilt = joystick.get_axis(1) * 20
        turret.set_orientation(pan, tilt)

        eventlet.sleep(0)
        clock.tick(50)


if __name__ == '__main__':
    main()
