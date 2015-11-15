#!/bin/bash
set -e

PKGLIST=(
    libeigen3-dev libsnappy-dev python-snappy
    libboost1.55-dev 
    libboost-system1.55-dev libboost-program-options1.55-dev 
    libboost-coroutine1.55-dev libboost-context1.55-dev 
    libboost-test1.55-dev libboost-python1.55-dev 
    libboost-date-time1.55-dev libboost-filesystem1.55-dev
)

if apt-get install --dry-run "${PKGLIST[@]}" | grep '^Conf'; then
    echo
    echo Need to install some packages
    (set -x;
        sudo apt-get install "${PKGLIST[@]}"
    )
else
    echo All packages up to date
fi


