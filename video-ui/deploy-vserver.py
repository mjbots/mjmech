#!/bin/bash
set -e
./vserver/vserver.py --check
rsync --exclude '*~' -a vserver odroid:/tmp/
ssh odroid -t exec /tmp/vserver/start.sh
