#!/bin/bash

# This file starts an instance of vserver.
# It is run either by vserver_launcher.py, or from ssh session during
# development.

# Kill all previous vserver instances
echo -n "Killing previous vservers: "
if killall -v vserver.py; then
    # Wait for servers to die (and camera to be released)
    sleep 5
fi

cd $(dirname $(readlink -f "$0")) || exit 1
echo Starting from `pwd` on `date`
exec ./vserver.py -s herkulex -c real.cfg -p /dev/ttyACM99
#EC=$?
#echo "[Exited with code $EC]"
#exit $EC
