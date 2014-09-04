#!/bin/bash

# This script is started by screen session on startup

mydir=$(dirname $(readlink -f "$0"))
cd $mydir || exit 1

if [ "$(cat ~/autostart-enabled)" != "yes" ]; then
    echo Autostart is not enabled
    exit 1
fi

PFILE=/tmp/vserver-process
echo $$ > $PFILE.loop-pid
exec 11>$PFILE.loop-lock
if ! flock -w 5 11; then
    echo Failed to get loop lock
    exit 1
fi


echo Getting ready to run
while sleep 1; do
    # If we have newer version in /tmp, use this instead
    if [[ -d /tmp/vserver ]]; then
        cd /tmp/vserver
    else
        cd $mydir
    fi
    echo Staring from `pwd` on `date` > /tmp/vserver.log
    cat /tmp/vserver.log
    ./start.sh loop 2>&1 | tee -a /tmp/vserver.log
    EC=${PIPESTATUS[0]}
    if [[ "$EC" == 143 ]]; then
        # terminated, restart now
        echo Process killed, restarting
    else
        # failed, restart after delay
        echo Failed with code $EC
        sleep 8
    fi
done
