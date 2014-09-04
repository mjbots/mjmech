#!/bin/bash
# Last component must be named 'vserver'
TMPDIR=/tmp/odroid-assemble/vserver

if [[ "$1" != "start" && "$1" != "install" && "$1" != "assemble" && "$1" != "exec" ]]; then
    cat <<EOF
Usage: $0 command
Commands:
  start     -- upload server to odroid's RAM and start
  install   -- install server to odroid's flash
  assemble  -- do not upload, just put all files to $TMPDIR
  exec CMD  -- upload, then exec CMD in vserver's dir
EOF
    exit 1
fi

set -e
cd $(dirname $(readlink -f "$0"))
RELEASE=$(lsb_release -sc)
if [[ "$RELEASE" != "precise" ]]; then
    # The program cannot even be imported on precise
    ./vserver/vserver.py --check
fi

mkdir -p $TMPDIR
RSCMD=(rsync -a --exclude '*~' --exclude '*.pyc' --delete )


"${RSCMD[@]}" --exclude=legtool --exclude=install-packages.sh \
    --exclude=gbulb --exclude=real.cfg vserver $TMPDIR/..
"${RSCMD[@]}" ../legtool ../install-packages.sh ../gbulb ../real.cfg $TMPDIR

if [[ "$1" == "assemble" ]]; then
    echo Assembled
    exit 0
fi

echo Uploading
rsync -a --delete $TMPDIR odroid:/tmp/
if [[ "$1" == "start" ]]; then
    echo Starting vserver
    ssh odroid -t exec /tmp/vserver/start.sh
elif [[ "$1" == "install" ]]; then
    echo Installing vserver to flash
    ssh odroid -t rsync -av --delete /tmp/vserver/ /home/odroid/vserver/
elif [[ "$1" = "exec" ]]; then
    shift
    set -x
    ssh odroid -t cd /tmp/vserver \&\& "$@"
fi
