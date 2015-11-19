#!/bin/bash
set -e

yes_flag=""
if [[ "$1" == "-y" ]]; then
    yes_flag="-y"
elif [[ "$1" != "" ]]; then
    echo invalid option
    exit 1
fi

PKGLIST=(
    scons libeigen3-dev libsnappy-dev python-snappy libi2c-dev
    pyside-tools libboost1.55-dev
    libboost-system1.55-dev libboost-program-options1.55-dev
    libboost-coroutine1.55-dev libboost-context1.55-dev
    libboost-test1.55-dev libboost-python1.55-dev
    libboost-date-time1.55-dev libboost-filesystem1.55-dev
)

if apt-get install --dry-run "${PKGLIST[@]}" | grep '^Conf'; then
    echo
    echo Need to install some packages
    (set -x;
        sudo apt-get install $yes_flag "${PKGLIST[@]}"
    )
else
    echo All packages up to date
fi
