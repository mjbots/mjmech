#!/bin/bash


# Build and run the tool
# (gcc is so fast we can just build it on every app start)
mydir=$(dirname $(readlink -f "$0"))

set -e
(cd $mydir &&
    gcc -Wall -Werror -g -o /tmp/i2c_poller.exe main.c i2c.c lsm_accel.c
)

if [[ `uname -m` != armv7l ]]; then
    echo This script will not run -- it is for odroid. But syntax is ok >&2
    exit 0
fi

exec /tmp/i2c_poller.exe "$@"