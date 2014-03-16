#!/bin/bash

cd $(dirname $(readlink -f "$0")) || exit 1

LPID=/tmp/vserver-loop.pid
if test -f $LPID  && kill -0 $(cat $LPID); then
    echo Another loop is running
    exit 1
fi

echo $$ > $LPID
echo Getting ready to run
while sleep 1; do
    date
    ./vserver.py 2>&1 | tee /tmp/vserver.log
    EC=${PIPESTATUS[0]}
    if [[ "$EC" == 143 ]]; then
        # terminated, restart now
        echo Process killed, restarting
    else
        echo Failed with code $EC
        sleep 20
    fi
done
