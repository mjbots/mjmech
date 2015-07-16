#!/usr/bin/python

# Copyright 2014-2015 Josh Pieper, jjp@pobox.com.
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


# TODO jpieper
#   * Make logging work even if a driver isn't currently active.


# Hack so that pygame won't barf in a headless environment.
import os
os.environ['SDL_VIDEODRIVER'] = 'dummy'

import capnp
import ConfigParser
import datetime
import functools
import logging
import optparse
import pygame
import smbus
import struct
import sys
import time
import thread

import trollius as asyncio
from trollius import From, Task, Return

sys.path.append('../legtool')

from legtool.async import trollius_trace
from legtool.servo import selector
from legtool.gait import leg_ik
from legtool.gait import ripple
from legtool.gait.common import Command

import telemetry_log

UPDATE_TIME = 0.001


def localpath(x):
    return os.path.join(sys.path[0], x)


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

    def __init__(self, idle_state, servo):
        self.gait = None
        self.servo = servo
        self.idle_state = idle_state
        self.state = self.idle_state.copy()

        self.mode = self.IDLE

        # This event should be set anytime a next_command is present.
        self.command_event = asyncio.Event()
        self.next_gait = None
        self.next_command = None

        now = datetime.datetime.utcnow()
        filename = '/tmp/data-%04d%02d%02d-%02d%02d%02d.log' % (
            now.year, now.month, now.day, now.hour, now.minute, now.second)

        self.log = telemetry_log.Writer(open(filename, 'w'))
        self.imu_log = self.log.register(
            'imu', open(localpath('imu_data.capnp')).read(), 'ImuData')
        self.servo_data = capnp.load('servo_data.capnp')
        self.servo_command_log = self.log.register(
            'servo_command', open(localpath('servo_data.capnp')).read(),
            'ServoData')
        self.gait_data = capnp.load('gait_data.capnp')
        self.gait_data_log = self.log.register(
            'gait_data', open(localpath('gait_data.capnp')).read(),
            'GaitData')
        self.joystick_data = capnp.load('joystick_data.capnp')
        self.joystick_data_log = self.log.register(
            'joystick', open(localpath('joystick_data.capnp')).read(),
            'JoystickData')

    def handle_imu(self, message):
        self.imu_log(message)

    @asyncio.coroutine
    def run(self):
        '''The run method starts gait driver operation.  Once invoked,
        it will begin commanding the servos.'''

        # It would be nice and/or cool to step the legs up into
        # position.  It may even be required for mechs where
        # instantaneous torque is insufficient to jump all the legs up
        # at once.  However, just forcing us to the gait's idle state
        # is easier for now.

        yield From(self.servo.enable_power(selector.POWER_ENABLE))

        self.state = self.idle_state.copy()

        do_idle = True

        while True:
            if do_idle:
                yield From(self._idle_state())

            # Run the current command as long as one is present.
            yield From(self._run_command())

            # Transition back to the idle state.
            do_idle = yield From(self._transition_to_idle())

    def set_command(self, gait, command):
        '''Command the mech to begin motion.

        :param gait: The gait object to use
        :param command: The current command to begin executing
        :type command: An instance of gait_command.Command
        '''

        self.next_gait = gait
        self.next_command = command
        self.command_event.set()

    def set_idle(self):
        '''Command the mech to assume an idle state where the legs are
        motionless.'''

        self.next_command = None
        self.next_gait = None
        self.command_event.clear()

        if self.mode == self.IDLE:
            pass
        elif self.mode == self.COMMAND:
            self.mode = self.IDLING

    @asyncio.coroutine
    def _idle_state(self):
        self.mode = self.IDLE
        self.state = self.idle_state.copy()
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
        assert self.next_gait is not None
        assert self.next_command is not None

        self.gait = self.next_gait
        try:
            self.gait.set_state(self.state, self.next_command)
        except ripple.NotSupported:
            print "Unsupported command, ignoring!"
            pass

        self.next_command = None

        yield From(self._run_while(
                lambda: self.mode == self.COMMAND,
                allow_new=True))

    def log_pose(self, state, id_to_deg_map):
        message = self.servo_data.ServoData.new_message()

        message.timestamp = time.time()
        message.init('values', len(id_to_deg_map))

        for index, (ident, deg) in enumerate(id_to_deg_map.iteritems()):
            message.values[index].id = ident
            message.values[index].angleDeg = deg

        self.servo_command_log(message)

        gait_data = self.gait_data.GaitData.new_message()

        gait_data.timestamp = message.timestamp

        def format_point(point, dest):
            dest.x = point.x
            dest.y = point.y
            dest.z = point.z

        def format_transform(transform, dest):
            format_point(transform.translation, dest.translation)
            dest.rotation.x = transform.rotation.x
            dest.rotation.y = transform.rotation.y
            dest.rotation.z = transform.rotation.z
            dest.rotation.w = transform.rotation.w

        format_transform(state.body_frame.transform_to_frame(state.robot_frame),
                         gait_data.bodyRobot)
        format_transform(state.cog_frame.transform_to_frame(state.robot_frame),
                         gait_data.cogRobot)
        gait_data.phase = state.phase
        for index, dest in [(0, gait_data.leg1Robot),
                            (1, gait_data.leg2Robot),
                            (2, gait_data.leg3Robot),
                            (3, gait_data.leg4Robot)]:
            format_point(
                state.legs[index].frame.map_to_frame(
                    state.robot_frame, state.legs[index].point),
                dest)

        self.gait_data_log(gait_data)


    @asyncio.coroutine
    def _run_while(self, condition, allow_new):
        current_time = time.time()

        while condition():
            if allow_new and self.next_command:
                if self.next_gait != self.gait:
                    self.command_event.clear()
                    self.mode = self.IDLING
                    raise Return()

                self.gait.set_command(self.next_command)
                self.next_gait = None
                self.next_command = None
                self.command_event.clear()

            next_time = time.time()
            delta = next_time - current_time
            self.state = self.gait.advance_time(delta)
            try:
                servo_pose = self.state.command_dict()
                self.log_pose(self.state, servo_pose)
                yield From(self.servo.set_pose(
                        servo_pose, pose_time=3 * UPDATE_TIME))
            except ripple.NotSupported:
                pass

            #delay = (current_time + UPDATE_TIME) - time.time()
            #if delay > 0.0:
            #    yield From(asyncio.sleep(delay))
            yield From(asyncio.sleep(UPDATE_TIME))
            current_time = next_time

    @asyncio.coroutine
    def _transition_to_idle(self):
        result = yield From(self.gait.transition_to_idle(
                lambda: (self.next_command is not None or
                         self.next_gait == self.gait),
                self._run_while))
        raise Return(result)


class ConfigRippleGait(object):
    def __init__(self, config, name, leg_ik_map):
        self.ripple_config = ripple.RippleConfig.read_settings(
            config, 'gaitconfig', leg_ik_map)

        self.gait = ripple.RippleGait(self.ripple_config)

    def get_idle_state(self):
        return self.gait.get_idle_state()

    def set_state(self, state, command):
        return self.gait.set_state(state, command)

    def set_command(self, command):
        return self.gait.set_command(command)

    def advance_time(self, delta_time):
        return self.gait.advance_time(delta_time)

    def is_command_pending(self):
        return self.gait.is_command_pending()

    @asyncio.coroutine
    def transition_to_idle(self, early_exit, run):
        # TODO jpieper: Higher levels have switched to getting "idle"
        # behavior by just commanding a zero lift_height_percent, and
        # thus this routine doesn't accomplish anything.
        #
        # Eventually, the "idle" concept should just be removed from
        # the GaitDriver entirely.
        raise Return(True)


class MechDriver(object):
    def __init__(self, options):
        self.options = options
        self.command_time = time.time()

        self.driver = None
        self.servo = None

        config = ConfigParser.ConfigParser()
        successful = config.read(options.config)
        assert successful, ('No config files found, searched: ' +
                            str(options.config))

        self.leg_ik_map = _load_ik(config)
        self.gait = ConfigRippleGait(config, 'gaitconfig', self.leg_ik_map)

        self.servo_name_map = {}
        if config.has_section('servo.names'):
            for name, value in config.items('servo.names'):
                self.servo_name_map[int(name)] = value

        self.default_command = {}
        if config.has_section('gaitconfig'):
            for key in ['command_body_x_mm',
                        'command_body_y_mm',
                        'command_body_z_mm',
                        'command_body_pitch_deg',
                        'command_body_roll_deg',
                        'command_body_yaw_deg']:
                if config.has_option('gaitconfig', key):
                    self.default_command[key.split('command_')[1]] = (
                        config.getfloat('gaitconfig', key))

    def create_default_command(self):
        result = Command()
        for key, value in self.default_command.iteritems():
            setattr(result, key, value)
        return result

    def handle_imu(self, message):
        if self.driver is None:
            return
        self.driver.handle_imu(message)

    @asyncio.coroutine
    def connect_servo(self):
        kwargs = {}
        if self.options.serial_port:
            kwargs['serial_port'] = self.options.serial_port
        if self.options.model_name:
            kwargs['model_name'] = self.options.model_name
        kwargs['servo_name_map'] = self.servo_name_map
        servo = yield From(selector.select_servo(
                self.options.servo,
                **kwargs))
        # Detect races
        assert self.servo is None, 'Trying to connect to servo twice'
        self.servo = servo

        self.driver = GaitDriver(self.gait.get_idle_state(), self.servo)

    @asyncio.coroutine
    def run(self):
        if self.servo is None:
            yield From(self.connect_servo())

        idle_task = Task(self._make_idle())
        driver_task = Task(self.driver.run())

        done, pending = yield From(
            asyncio.wait([idle_task, driver_task],
                         return_when=asyncio.FIRST_EXCEPTION))

        for x in done:
            x.result()

    def set_command(self, command):
        self.command_time = time.time()
        self.driver.set_command(self.gait, command)

    def set_idle(self):
        self.command_time = time.time()
        self.driver.set_idle()

    @asyncio.coroutine
    def _make_idle(self):
        while True:
            now = time.time()
            if now - self.command_time > 2.0:
                self.driver.set_idle()
            yield From(asyncio.sleep(0.5))

    @staticmethod
    def add_options(parser):
        parser.add_option('-c', '--config', help='configuration to use',
                          default=[])
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


_DEADBAND = 0.20


def remove_deadband(value):
    if abs(value) < _DEADBAND:
        return 0.0
    if value > 0.0:
        return (value - _DEADBAND) / (1.0 - _DEADBAND)
    return (value + _DEADBAND) / (1.0 - _DEADBAND)


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

    command = driver.create_default_command()
    idle = True

    values = {}

    while True:
        pygame.event.pump()

        events = pygame.event.get()
        for event in events:
            if event.type == pygame.JOYAXISMOTION:
                values[event.axis] = event.value

        x_speed = remove_deadband(values.get(0, 0.0))
        y_speed = -remove_deadband(values.get(1, 0.0))
        rot_speed = remove_deadband(values.get(3, 0.0))
        z_height = -remove_deadband(values.get(4, 0.0))

        #print time.time(), x_speed, y_speed, rot_speed, z_height

        command.translate_x_mm_s = x_speed * 100
        command.translate_y_mm_s = y_speed * 250
        command.rotate_deg_s = rot_speed * 30
        command.body_z_mm = z_height * 25

        if x_speed != 0.0 or y_speed != 0.0 or rot_speed != 0.0:
            driver.set_command(command)
            idle = False
        else:
            if not idle:
                driver.set_idle()
                idle = True

        if driver.driver:
            jdata = driver.driver.joystick_data.JoystickData.new_message()
            jdata.timestamp = time.time()
            jdata.translateXMmS = command.translate_x_mm_s
            jdata.translateYMmS = command.translate_y_mm_s
            jdata.rotateDps = command.rotate_deg_s
            jdata.bodyZMm = command.body_z_mm

            driver.driver.joystick_data_log(jdata)

        yield From(asyncio.sleep(0.001))
        clock.tick(100)

def imu_thread(loop, gait_driver):
    try:
        sm = smbus.SMBus(4)
    except:
        return

    def gyro_read(addr, size):
        return sm.read_i2c_block_data(0x6b, addr, size)

    def gyro_write(addr, data):
        sm.write_i2c_block_data(0x6b, addr, data)

    def to_int16(msb, lsb):
        result = msb * 256 + lsb
        if result > 32767:
            result = result - 65536
        return result

    def accel_read(addr, size):
        return sm.read_i2c_block_data(0x1d, addr, size)

    def accel_write(addr, data):
        sm.write_i2c_block_data(0x1d, addr, data)

    try:
        whoami = gyro_read(0x0f, 1)[0]
        if whoami != 0xd7:
            raise RuntimeError('L3GD20 whomai %02x != D7' % whoami)
    except Exception as e:
        print 'Could not initialize gyro:', str(e)
        return

    try:
        accel_read(0x20, 1)
    except Exception as e:
        print 'Could not initialize accelerometer:', str(e)
        return

    print 'initializing gyro'

    # Configure the gyroscope.
    gyro_write(0x20,
               [0x3f, # ODR=95Hz, Cut-off = 25Hz
                ])
    gyro_write(0x23,
               [0x80, # BDU=1, BLE=0, FS=0
                ])

    print 'initializing accelerometer'
    accel_write(0x20,
                [0b01010111, # ODR=100Hz, low_power=off all enabled
                 ])
    accel_write(0x23,
                [0b10101000, # BDU=(block), BLE=off, FS=11 (8g) HR=1
                 ])

    imu_data = capnp.load('imu_data.capnp')

    print 'reading imu'

    while True:
        # Read gyro values until we get a new one.
        for i in range(15):
            gdata = gyro_read(0xa7, 1)
            if gdata[0] & 0x08 != 0:
                break

        gdata = gyro_read(0xa7, 7)

        # Then read an accelerometer value.
        adata = accel_read(0xa7, 7)

        timestamp = time.time()

        # TODO jpieper: Transform this into a preferred coordinate
        # system.

        message = imu_data.ImuData.new_message()
        message.timestamp = time.time()
        # For a 250dps full scale range
        GSENSITIVITY = 0.00875
        message.xRateDps = to_int16(gdata[2], gdata[1]) * GSENSITIVITY
        message.yRateDps = to_int16(gdata[4], gdata[3]) * GSENSITIVITY
        message.zRateDps = to_int16(gdata[6], gdata[5]) * GSENSITIVITY

        G = 9.80665
        ASENSITIVITY = 0.004 / 64
        message.xAccelMps2 = to_int16(adata[2], adata[1]) * ASENSITIVITY * G
        message.yAccelMps2 = to_int16(adata[4], adata[3]) * ASENSITIVITY * G
        message.zAccelMps2 = to_int16(adata[6], adata[5]) * ASENSITIVITY * G

        loop.call_soon_threadsafe(functools.partial(
                gait_driver.handle_imu, message))
        time.sleep(0.005)


@asyncio.coroutine
def start(options):
    gait_driver = MechDriver(options)
    driver_task = Task(gait_driver.run())
    input_task = Task(handle_input(gait_driver))

    # Kick off the I2C IMU sampling thread.
    thread.start_new_thread(
        imu_thread, (asyncio.get_event_loop(), gait_driver))

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
