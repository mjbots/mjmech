#!/bin/bash
set -e
./vserver/vserver.py --check
rsync --exclude '*~' -a --delete vserver odroid:/tmp/
if [[ "$1" == "" ]]; then
    echo Staring vserver
    ssh odroid -t exec /tmp/vserver/start.sh
elif [[ "$1" == "install" ]]; then
    echo Installing vserver to flash
    ssh odroid -t rsync -av --delete /tmp/vserver/ /home/odroid/vserver/
else
    set -x
    ssh odroid -t cd /tmp/vserver \&\& "$@"
fi
