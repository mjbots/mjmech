#!/bin/bash
set -e
./vserver/vserver.py --check
rsync --exclude '*~' -a vserver odroid:/tmp/
if [[ "$1" == "" ]]; then
    echo Staring vserver
    ssh odroid -t exec /tmp/vserver/start.sh
else
    set -x
    ssh odroid -t cd /tmp/vserver \&\& "$@"
fi
