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

'''Routines for commanding and monitoring Dongbu HerkuleX servos.'''

import trollius as asyncio
from trollius import From, Return
import logging
import threading

from ..async import asyncio_serial

from . import port

logger = logging.getLogger(__name__)

class ChecksumError(RuntimeError):
    def __init__(self, packet):
        super(ChecksumError, self).__init__()
        self.packet = packet


class Packet(object):
    servo = None
    cmd = None
    data = None
    cksum_good = None

    def __str__(self):
        return repr((self.__class__.__name__, self.servo, self.cmd, self.data,
                     self.cksum_good))

class Response(object):
    indent = 0
    def __repr__(self):
        Response.indent += 2
        indent = ' ' * Response.indent
        result = ('<' + self.__class__.__name__+ '\n' +
                  '\n'.join([indent + '%s=%s' % (key, getattr(self, key))
                             for key in sorted(self.__dict__.keys())]) +
                  '  >\n')
        Response.indent -= 2
        return result


class StatusResponse(Response):
    def __init__(self, reg48, reg49):
        self.exceed_input_voltage_limit = (reg48 & 0x01) != 0
        self.exceed_allowed_pot_limit = (reg48 & 0x02) != 0
        self.exceed_temperature_limit = (reg48 & 0x04) != 0
        self.invalid_packet = (reg48 & 0x08) != 0
        self.overload_detected = (reg48 & 0x10) != 0
        self.driver_fault_detected = (reg48 & 0x20) != 0
        self.eep_reg_distorted = (reg48 & 0x40) != 0

        self.moving = (reg49 & 0x01) != 0
        self.inposition = (reg49 & 0x02) != 0
        self.checksum_error = (reg49 & 0x04) != 0
        self.unknown_command = (reg49 & 0x08) != 0
        self.exceed_reg_range = (reg49 & 0x10) != 0
        self.garbage_detected = (reg49 & 0x20) != 0
        self.motor_on = (reg49 & 0x40) != 0

        self.reg48 = reg48
        self.reg49 = reg49

    def active_flags(self):
        """Return list of short strings, one for each flag which is set"""
        result = list()
        for attr in (
            "exceed_input_voltage_limit", "exceed_allowed_pot_limit",
            "exceed_temperature_limit", "invalid_packet",
            "overload_detected", "driver_fault_detected",
            "eep_reg_distorted",

            "moving", "inposition", "checksum_error", "unknown_command",
            "exceed_reg_range", "garbage_detected"):
            if getattr(self, attr):
                result.append(attr)
        # 'motor_on' is enabled all the time while operating; to keep statuses
        #  short, hide it and use 'motor_off' instead
        if not self.motor_on:
            result.append("motor_off")
        return result

    def has_errors(self):
        """Return True if any of user-resettable error flags are set.
        This should function should return False after clear_errors"""
        return ((self.reg48 & 0x7F) != 0 or
                (self.reg49 & 0x3C) != 0)


class MemReadResponse(Response):
    def __init__(self, data):
        self.register_start = data[0]
        self.length = data[1]
        self.status = StatusResponse(*data[-2:])
        self.data = data[2:-2]


class HerkuleX(object):
    """Provides an interface to the serial based HerkuleX line of
    servos."""

    (CMD_EEP_WRITE,
     CMD_EEP_READ,
     CMD_RAM_WRITE,
     CMD_RAM_READ,
     CMD_I_JOG,
     CMD_S_JOG,
     CMD_STAT,
     CMD_ROLLBACK,
     CMD_REBOOT) = range(1, 10)

    BROADCAST = 0xfe
    PERIOD_MS = 11.2

    LED_GREEN = 0x01
    LED_BLUE = 0x02
    LED_RED = 0x04

    EEP_ID = 6
    REG_DEAD_ZONE = 10
    REG_TORQUE_CONTROL = 52

    def __init__(self, serial_port):
        self.baud_rate = 115200
        self.serial_port_name = serial_port
        self.serial = None

    @asyncio.coroutine
    def start(self):
        self.serial = yield From(port.open(self.serial_port_name))

    @asyncio.coroutine
    def enumerate(self):
        """Enumerate the list of servos on the bus.  Note, this will
        take approximately 5s to complete.

        :returns: a list of integer servo IDs
        """
        result = []
        for servo in range(0xfe):
            try:
                yield From(asyncio.wait_for(self.status(servo), 0.02))
                result.append(servo)
            except asyncio.TimeoutError:
                pass

        raise Return(result)

    def _cksum1(self, size, servo, cmd, data):
        return (size ^ servo ^ cmd ^
                reduce(lambda a, b: a ^ b, [ord(x) for x in data], 0x00))

    @asyncio.coroutine
    def flush(self):
        with (yield From(self.serial.read_lock)):
            yield From(self.serial.flush())

    @asyncio.coroutine
    def _raw_send_packet(self, servo, cmd, data):
        assert servo >= 0 and servo <= 0xfe
        assert cmd >= 0 and cmd <= 0xff

        packet_size = 7 + len(data)
        cksum1 = self._cksum1(packet_size, servo, cmd, data)
        cksum2 = cksum1 ^ 0xff
        to_write = ('\xff\xff' + chr(7 + len(data)) + chr(servo) + chr(cmd) +
                    chr(cksum1 & 0xfe) + chr(cksum2 & 0xfe)) + data

        yield From(self.serial.write(to_write))

    @asyncio.coroutine
    def send_packet(self, servo, cmd, data):
        with (yield From(self.serial.write_lock)):
            yield From(self._raw_send_packet(servo, cmd, data))

    @asyncio.coroutine
    def _raw_recv_packet(self):

        @asyncio.coroutine
        def read_frame_header():
            # Read until we get a frame header.
            maybe_header = ''
            while maybe_header != '\xff\xff':
                maybe_header += yield From(self.serial.read(
                        max(1, min(2, 2 - len(maybe_header)))))
                if len(maybe_header) > 2:
                    maybe_header = maybe_header[-2:]

        yield From(asyncio.wait_for(
                read_frame_header(),
                max(0.1, 6 * 10 / self.baud_rate + 0.01)))

        header = yield From(asyncio.wait_for(
                self.serial.read(5), max(0.1, 5*10 / self.baud_rate + 0.005)))
        size, servo, cmd, cksum1, cksum2 = [ord(x) for x in header]

        data = yield From(asyncio.wait_for(
                self.serial.read(size - 7),
                max(0.1, (size - 7) * 10 / self.baud_rate + 0.005)))

        expected_cksum1 = self._cksum1(size, servo, cmd, data)
        expected_cksum2 = expected_cksum1 ^ 0xff

        result = Packet()
        result.servo = servo
        result.cmd = cmd
        result.data = data
        result.cksum_good = (cksum1 == (expected_cksum1 & 0xfe) and
                             cksum2 == (expected_cksum2 & 0xfe))

        if not result.cksum_good:
            raise ChecksumError(result)

        raise Return(result)

    @asyncio.coroutine
    def recv_packet(self):
        with (yield From(self.serial.read_lock)):
            result = yield From(self._raw_recv_packet())
            raise Return(result)

    @asyncio.coroutine
    def send_recv_packet(self, servo, cmd, data):
        with (yield From(self.serial.write_lock)):
            with (yield From(self.serial.read_lock)):
                yield From(self._raw_send_packet(servo, cmd, data))
                result = yield From(self._raw_recv_packet())
                raise Return(result)

    @asyncio.coroutine
    def mem_read(self, cmd, servo, reg, length):
        assert cmd >= 0 and cmd <= 0xff
        assert reg >= 0 and reg <= 0xff
        assert length >= 0 and length <= 0x7f

        received = yield From(self.send_recv_packet(
                servo, cmd, chr(reg) + chr(length)))
        if received.servo != servo and received.servo != self.BROADCAST:
            yield From(self.serial.flush())
            raise RuntimeError(
                'synchronization error detected, sent '
                'request for servo %d, response from %d, flushed' %
                (servo, received.servo))
        assert received.cmd == (0x40 | cmd)
        assert len(received.data) == length + 4

        raise Return(MemReadResponse([ord(x) for x in received.data]))

    @asyncio.coroutine
    def mem_write(self, cmd, servo, reg, data):
        assert cmd >= 0 and cmd <= 0xff
        assert reg >= 0 and reg <= 0xff
        assert len(data) <= 0x7f
        for x in data:
            assert x >= 0 and x <= 0xff

        yield From(self.send_packet(servo, cmd,
                                    chr(reg) + chr(len(data)) +
                                    ''.join([chr(x) for x in data])))

    @asyncio.coroutine
    def eep_read(self, servo, reg, length):
        result = yield From(
            self.mem_read(self.CMD_EEP_READ, servo, reg, length))
        raise Return(result)

    @asyncio.coroutine
    def eep_write(self, servo, reg, data):
        yield From(self.mem_write(self.CMD_EEP_WRITE, servo, reg, data))

    @asyncio.coroutine
    def ram_read(self, servo, reg, length):
        result = yield From(
            self.mem_read(self.CMD_RAM_READ, servo, reg, length))
        raise Return(result)

    @asyncio.coroutine
    def ram_write(self, servo, reg, data):
        yield From(self.mem_write(self.CMD_RAM_WRITE, servo, reg, data))

    @asyncio.coroutine
    def clear_errors(self, servo):
        yield From(self.ram_write(servo, 48, [0, 0]))

    @asyncio.coroutine
    def i_jog(self, targets):
        """Set the position of one or more servos with unique play
        times for each servo.

        :param targets: list of servos, positions, and times
        :type targets: list of tuples (servo, position, led, time_ms)
        """
        for target in targets:
            assert target[0] >= 0 and target[0] <= 0xff
            assert target[1] >= 0 and target[1] <= 0x7fff
            assert target[2] >= 0 and target[2] <= 7
            assert target[3] >= 0 and target[3] <= (self.PERIOD_MS * 255)

        def build_frame(target):
            return (chr(target[1] & 0xff) +
                    chr((target[1] >> 8) & 0xff) +
                    chr(target[2] << 2) +
                    chr(target[0]) +
                    chr(int(target[3] / self.PERIOD_MS)))

        servo = self.BROADCAST if len(targets) != 1 else targets[0][0]

        yield From(self.send_packet(
                servo, self.CMD_I_JOG,
                ''.join([build_frame(x) for x in targets])))

    @asyncio.coroutine
    def s_jog(self, time_ms, targets):
        """Set the position of one or more servos with a common play
        time across all servos.

        :param targets: list of servos and positions
        :type targets: list of tuples (servo, position, led)
        """
        assert time_ms >= 0 and time_ms <= (self.PERIOD_MS * 255)

        for target in targets:
            assert target[0] >= 0 and target[0] <= 0xff
            assert target[1] >= 0 and target[1] <= 0x7fff
            assert target[2] >= 0 and target[2] <= 0x7f

        servo = self.BROADCAST if len(targets) != 1 else targets[0][0]

        def build_frame(target):
            return (chr(target[1] & 0xff) +
                    chr((target[1] >> 8) & 0xff) +
                    chr(target[2] << 2) +
                    chr(target[0]))

        yield From(self.send_packet(
                servo, self.CMD_S_JOG,
                chr(int(time_ms / 11.2)) +
                ''.join(build_frame(x) for x in targets)))

    @asyncio.coroutine
    def status(self, servo):
        received = yield From(self.send_recv_packet(servo, self.CMD_STAT, ''))
        assert ((received.servo == servo or servo == self.BROADCAST) and
                (received.cmd == (0x40 | self.CMD_STAT)) and
                (len(received.data) == 2)), (
            "Received wrong STATUS response from servo %d: %s" % (servo, received))

        reg48 = ord(received.data[0])
        reg49 = ord(received.data[1])

        result = StatusResponse(reg48, reg49)

        raise Return(result)

    @asyncio.coroutine
    def clear_status(self, servo):
        yield From(self.ram_write(servo, 48, [0, 0]))

    @asyncio.coroutine
    def reboot(self, servo):
        yield From(self.send_packet(servo, self.CMD_REBOOT, ''))

    @asyncio.coroutine
    def set_leds(self, servo, leds):
        """@p leds is bitflag of LED_RED, LED_GREEN, LED_BLUE"""
        yield From(self.ram_write(servo, 53, [leds]))

    @asyncio.coroutine
    def temperature_C(self, servo):
        try:
            data = yield From(self.ram_read(servo, 55, 1))
            value = data.data[0]

            # Note, this formula was derived from the Dongbu lookup table,
            # and becomes terribly inaccurate below -20C.
            raise Return((value - 40) * 0.5125 - 19.38)
        except asyncio.TimeoutError:
            raise Return(None)

    @asyncio.coroutine
    def voltage(self, servo):
        try:
            data = yield From(self.ram_read(servo, 54, 1))
            value = data.data[0]

            raise Return(0.074 * value)
        except asyncio.TimeoutError:
            raise Return(None)

    @asyncio.coroutine
    def position(self, servo):
        try:
            # NOTE: The datasheet appears to be off here.
            data = yield From(self.ram_read(servo, 60, 2))
            value = data.data
            if value is None:
                raise Return(None)
            raise Return(value[1] << 8 | value[0])
        except asyncio.TimeoutError:
            raise Return(None)

    @asyncio.coroutine
    def pwm(self, servo):
        try:
            # NOTE: The datasheet says this should be at RAM register 62,
            # however, nothing much ever shows up there, and something
            # which looks a lot like PWM is at the reserved RAM address
            # 64.
            data = yield From(self.ram_read(servo, 64, 2))
            value = data.data
            if value is None:
                raise Return(None)
            result = value[1] << 8 | value[0]
            if result > 32767:
                result = result - 65536
            raise Return(result)
        except asyncio.TimeoutError:
            raise Return(None)

    @asyncio.coroutine
    def set_position_kp(self, servo, value):
        yield From(self.ram_write(
                servo, 24, [ value & 0xff, (value >> 8) & 0xff ]))

    @asyncio.coroutine
    def set_position_kd(self, servo, value):
        yield From(self.ram_write(
                servo, 26, [ value & 0xff, (value >> 8) & 0xff ]))

    @asyncio.coroutine
    def set_position_ki(self, servo, value):
        yield From(
            self.ram_write(servo, 28, [ value & 0xff, (value >> 8) & 0xff ]))
