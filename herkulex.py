#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import eventlet
import logging
import serial
import threading

logger = logging.getLogger(__name__)

class ChecksumError(RuntimeError):
    def __init__(self, packet):
        super(ChecksumError, self).__init__()
        self.packet = packet

class EventletSerial(object):
    def __init__(self, raw_serial):
        self.raw_serial = raw_serial
        self.sem = eventlet.semaphore.Semaphore()

    def write(self, data):
        assert self.sem.locked()
        logger.debug('writing:' + ' '.join(['%02x' % ord(x) for x in data]))
        to_write = data
        while len(to_write):
            eventlet.hubs.trampoline(self.raw_serial.fileno(), write=True)
            written = self.raw_serial.write(to_write)
            to_write = to_write[written:]

    def read(self, size):
        assert self.sem.locked()
        bytes_remaining = size
        read_so_far = ""
        while bytes_remaining > 0:
            eventlet.hubs.trampoline(self.raw_serial.fileno(), read=True)
            this_read = self.raw_serial.read(bytes_remaining)
            bytes_remaining -= len(this_read)
            read_so_far += this_read

        return read_so_far

class Packet(object):
    servo = None
    cmd = None
    data = None
    cksum_good = None

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

    REG_TORQUE_CONTROL = 52

    def __init__(self, serial_port):
        self.raw_serial = serial.Serial(
            port=serial_port, baudrate=115200, timeout=0)

        self.baud_rate = 115200
        self.serial = EventletSerial(self.raw_serial)
        self._port_sem = eventlet.semaphore.Semaphore(1)

    def enumerate(self):
        """Enumerate the list of servos on the bus.  Note, this will
        take approximately 5s to complete.  Spawn a new eventlet
        greenthread if you can't block that long.

        :returns: a list of integer servo IDs
        """
        result = []
        for servo in range(0xfe):
            with eventlet.timeout.Timeout(0.02, false):
                self.status(servo)
                result.append(servo)

        return result

    def _cksum1(self, size, servo, cmd, data):
        return (size ^ servo ^ cmd ^
                reduce(lambda a, b: a ^ b, [ord(x) for x in data], 0x00))

    def _raw_send_packet(self, servo, cmd, data):
        assert servo >= 0 and servo <= 0xfe
        assert cmd >= 0 and cmd <= 0xff

        packet_size = 7 + len(data)
        cksum1 = self._cksum1(packet_size, servo, cmd, data)
        cksum2 = cksum1 ^ 0xff
        to_write = ('\xff\xff' + chr(7 + len(data)) + chr(servo) + chr(cmd) +
                    chr(cksum1 & 0xfe) + chr(cksum2 & 0xfe)) + data

        self.serial.write(to_write)

    def send_packet(self, servo, cmd, data):
        with self.serial.sem:
            self._raw_send_packet(servo, cmd, data)

    def _raw_recv_packet(self):
        with eventlet.timeout.Timeout(max(0.1, 6*10 / self.baud_rate + 0.01)):
            # Read until we get a frame header.
            maybe_header = ''
            while maybe_header != '\xff\xff':
                maybe_header += self.serial.read(
                    max(1, min(2, 2 - len(maybe_header))))
                if len(maybe_header) > 2:
                    maybe_header = maybe_header[-2:]

        with eventlet.timeout.Timeout(max(0.1, 5*10 / self.baud_rate + 0.005)):
            # Got a header.  Now read the rest.
            size, servo, cmd, cksum1, cksum2 = \
                [ord(x) for x in self.serial.read(5)]

        with eventlet.timeout.Timeout(max(0.1, (size - 7) * 10 /
                                          self.baud_rate + 0.005)):
            data = self.serial.read(size - 7)

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

        return result

    def recv_packet(self):
        with self.serial.sem:
            return self._raw_recv_packet()

    def send_recv_packet(self, servo, cmd, data):
        with self.serial.sem:
            self._raw_send_packet(servo, cmd, data)
            return self._raw_recv_packet()


    def mem_read(self, cmd, servo, reg, length):
        assert cmd >= 0 and cmd <= 0xff
        assert reg >= 0 and reg <= 0xff
        assert length >= 0 and length <= 0x7f

        received = self.send_recv_packet(servo, cmd, chr(reg) + chr(length))
        assert received.servo == servo or servo == self.BROADCAST
        assert received.cmd == (0x40 | cmd)
        assert len(received.data) == length + 4

        return MemReadResponse([ord(x) for x in received.data])

    def mem_write(self, cmd, servo, reg, data):
        assert cmd >= 0 and cmd <= 0xff
        assert reg >= 0 and reg <= 0xff
        assert len(data) <= 0x7f
        for x in data:
            assert x >= 0 and x <= 0xff

        self.send_packet(servo, cmd,
                         chr(reg) + chr(len(data)) +
                         ''.join([chr(x) for x in data]))

    def eep_read(self, servo, reg, length):
        return self.mem_read(self.CMD_EEP_READ, servo, reg, length)

    def eep_write(self, servo, reg, data):
        self.mem_write(self.CMD_EEP_WRITE, servo, reg, data)

    def ram_read(self, servo, reg, length):
        return self.mem_read(self.CMD_RAM_READ, servo, reg, length)

    def ram_write(self, servo, reg, data):
        self.mem_write(self.CMD_RAM_WRITE, servo, reg, data)

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

        self.send_packet(
            servo, self.CMD_I_JOG,
            ''.join([build_frame(x) for x in targets]))

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

        self.send_packet(
            servo, self.CMD_S_JOG,
            chr(int(time_ms / 11.2)) +
            ''.join(build_frame(x) for x in targets))

    def status(self, servo):
        received = self.send_recv_packet(servo, self.CMD_STAT, '')
        assert received.servo == servo or servo == self.BROADCAST
        assert received.cmd == (0x40 | self.CMD_STAT)
        assert len(received.data) == 2

        reg48 = ord(received.data[0])
        reg49 = ord(received.data[1])

        result = StatusResponse(reg48, reg49)

        return result

    def reboot(self, servo):
        self.send_packet(servo, self.CMD_REBOOT, '')

    def temperature_C(self, servo):
        try:
            value = self.ram_read(servo, 55, 1).data[0]

            # Note, this formula was derived from the Dongbu lookup table,
            # and becomes terribly inaccurate below -20C.
            return (value - 40) * 0.5125 - 19.38
        except eventlet.timeout.Timeout:
            return None

    def position(self, servo):
        try:
            # NOTE: The datasheet appears to be off here.
            value = self.ram_read(servo, 60, 2).data
            if value is None:
                return None
            return value[1] << 8 | value[0]
        except eventlet.timeout.Timeout:
            return None

    def pwm(self, servo):
        try:
            # NOTE: The datasheet says this should be at RAM register 62,
            # however, nothing much ever shows up there, and something
            # which looks a lot like PWM is at the reserved RAM address
            # 64.
            value = self.ram_read(servo, 64, 2).data
            if value is None:
                return None
            result = value[1] << 8 | value[0]
            if result > 32767:
                result = result - 65536
            return result
        except eventlet.timeout.Timeout:
            return None



if __name__ == '__main__':
    port = HerkuleX('/dev/ttyUSB0')
    port.reboot(port.BROADCAST)
