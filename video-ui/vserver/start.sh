#!/bin/bash
# This file starts and runs a single instance vserver
# It is run either from loop.sh on startup, or from ssh session during 
# development.
PFILE=/tmp/vserver-process

if [[ "$1" != "loop" ]]; then
    # Kill a loop, if any.
    # Once you start process manually, you have to do it until reboot.
    exec 11>$PFILE.loop-lock
    if ! flock -n 11; then
        echo Killing startup loop
        kill $(cat $PFILE.loop-pid)
        if ! flock -w 10 11; then
            echo Loop does not want to quit
            exit 1
        fi
    fi
fi

# Kill old process, if any
vs_pid="$(cat $PFILE.pid 2>/dev/null)"
if [[ "$vs_pid" != "" ]]; then
    echo Another instance is running, killing it
    echo old process: $(ps --no-headers -l -p $vs_pid)
    kill $vs_pid
    # We will know when process dies when the lock is released
fi

# Get a lock to run the process
exec 9>$PFILE.lock
if ! flock -w 1 9; then
    echo Waiting for other process to exit
    if ! flock -w 30 9; then
        echo Other process failed to exit
        exit 1
    fi
fi

# Start the process
cd $(dirname $(readlink -f "$0")) || exit 1
echo Starting from `pwd` on `date`
./vserver.py -s herkulex -c real.cfg -p /dev/ttyACM99
EC=$?
echo "*** Exited with code $EC ***"
exit $EC
