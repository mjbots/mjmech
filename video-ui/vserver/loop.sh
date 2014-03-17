#!/bin/bash

mydir=$(dirname $(readlink -f "$0"))
cd $mydir || exit 1

LPID=/tmp/vserver-loop.pid
if test -f $LPID  && kill -0 $(cat $LPID); then
    echo Another loop is running
    exit 1
fi

echo $$ > $LPID
echo Getting ready to run
while sleep 1; do
    # If we have newer version in /tmp, use this instead
    if [[ -d /tmp/vserver ]]; then
        cd /tmp/vserver
    fi
    echo Staring from `pwd` on `date` > /tmp/vserver.log
    cat /tmp/vserver.log
    ./vserver.py 2>&1 | tee -a /tmp/vserver.log
    EC=${PIPESTATUS[0]}
    if [[ "$EC" == 143 ]]; then
        # terminated, restart now
        echo Process killed, restarting
    else
        echo Failed with code $EC
        sleep 8
    fi
done
