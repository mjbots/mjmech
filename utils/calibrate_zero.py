#!/usr/bin/python3 -B

# Copyright 2019 Josh Pieper, jjp@pobox.com.
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

import argparse
import subprocess
import time


def _stream_command(stdin, stdout, data):
    print("cmd: ", data)
    stdin.write(data.encode('utf8') + b'\r\n')
    stdin.flush()

    result = []
    while True:
        line = stdout.readline()

        print("got line:", line)
        if line.startswith(b'OK\r') or line.startswith(b'OK\n'):
            return result
        stripped = line.strip()
        if not stripped:
            continue
        result.append(line.decode('utf8').strip())


def _parse_data(data):
    result = {}
    for line in data:
        key, value = line.split(' ')
        result[key] = value

    print("parse_data:", result)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--target', required=True)

    args = parser.parse_args()

    mp = subprocess.Popen(
        ['./multiplex_tool',
         '--type', 'serial',
         '--serial_port', '/dev/ttyAMA0',
         '--serial_baud', '3000000',
         '-c',
         '-t', args.target,],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE)

    def command(data):
      return _stream_command(mp.stdin, mp.stdout, data)

    command('tel stop')
    # TODO discard stuff

    # Now we can begin.
    command("tel text")
    servo_stats = _parse_data(command("tel get servo_stats"))
    command("conf set motor.position_offset {}".format(
        -int(servo_stats['servo_stats.position_raw'])))
    command("conf write")
    command("d rezero")


if __name__ == '__main__':
    main()
