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
    libboost-serialization1.55-dev
    libboost-date-time1.55-dev libboost-filesystem1.55-dev
)

if [[ "$yes_flag" != "" ]] || (
        apt-get install --dry-run "${PKGLIST[@]}" \
            | egrep ' could not be installed| ^Conf'
    ); then
    echo
    echo Need to install some packages
    (set -x;
        sudo apt-get install $yes_flag "${PKGLIST[@]}"
    )
else
    echo All packages up to date
fi

gstreamer_root='/opt/gstreamer-1.4.5'
gs_min_revision=1
revision="$(cat $gstreamer_root/mj-gstreamer-revision || echo not-found)"

if ! expr "(" "$revision" + 0 ")" ">=" "$gs_min_revision" >/dev/null; then
    echo "Need to install gstreamer into $gstreamer_root"

    if [[ "$yes_flag" == "" ]]; then
        echo -n 'Continue? (y/n) '
        read response
        if [[ "$response" != "y" ]]; then
            echo cancelled
            exit 1
        fi
    fi

    tar_name=mj-$(basename $gstreamer_root)-
    tar_name+=$(uname -m)-$(lsb_release -c -s).tar.bz2

    # Note: if wget fails for some reason, just copy the tar file into /tmp
    tmp_tar_name="/tmp/$tar_name"
    if [[ ! -f "$tmp_tar_name" ]]; then
        echo Fetching gstreamer tar...
        url=https://raw.githubusercontent.com/mjbots/mj-gstreamer-build
        url+=/master/tars/$tar_name
        (set -x;
            wget -O "$tmp_tar_name.partial" "$url"
            mv "$tmp_tar_name.partial" "$tmp_tar_name"
        )
    fi

    echo Verifying gstreamer tar $tmp_tar_name ...
    gstreamer_root_rel=${gstreamer_root#/}
    revision="$(tar xOf "$tmp_tar_name" \
                    "$gstreamer_root_rel/mj-gstreamer-revision")"
    echo " .. revision found: '$revision'"
    if ! expr "(" "$revision" + 0 ")" ">=" "$gs_min_revision" >/dev/null; then
        echo Incorrect revision
        exit 1
    fi

    echo Unpacking gstreamer tar...
    # move old directory out-of-the-way
    if test -d $gstreamer_root; then
        echo renaming old directory
        (set -x;
            sudo mv $gstreamer_root \
                $gstreamer_root.$(date -r $gstreamer_root +%F-%H%M%S)
        )
    fi

    # This .tar file starts at disk root, but we only extreact files under
    # gstreamer_root for security.
    (set -x;
        sudo tar xf "$tmp_tar_name" -C / "$gstreamer_root_rel"
    )
fi
