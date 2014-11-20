#!/bin/bash

if [[ "$1" != "-g" ]]; then
    cat <<EOF
Usage:
    $0 -g

This file installs/updates startup scripts to odroid device as required.
EOF
    exit 1
fi

DESTUSER=odroid

if [[ `id -un` != $DESTUSER ]]; then
    echo Error: this must be run under user $DESTUSER
    exit 1
fi

set -e

(set -x;
    rsync -tv vserver_launcher.py /home/$DESTUSER/
    sudo rsync -tv vserver-launcher.conf /etc/init
    sudo initctl reload-configuration
)

echo Success