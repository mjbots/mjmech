#!/bin/bash

if [[ "$1" != "start" && "$1" != "fg-start" && "$1" != "assemble" && \
      "$1" != "logs" && "$1" != "stop" && "$1" != "exec" ]]; then
    cat <<EOF
Usage: $0 command
Commands:
  start     -- upload server to odroid's RAM and start in background.
               Connection will be kept open (to receive errors), but may be
               closed at any time.
  logs      -- Get recent/follow new logs from background process
  stop      -- Kill background process
  fg-start  -- upload server to odroid's RAM and start in foreground.
               Server will be killed if Ctrl+C is pressed.
  install   -- install launcher and startup script
  assemble  -- do not upload, just put all files to $TMPDIR
  exec CMD  -- upload, then exec CMD in vserver's dir, with X11 fwd. Example:
   exec sudo ./install-packages.sh
   exec legtool/legtool.py
   exec legtool/herkulex_tool.py -d /dev/ttyACM99 -b -a 2
   exec ./install-startup-scripts.sh -g
   exec ./send-video.sh --test
EOF
    exit 1
fi

set -e
cd $(dirname $(readlink -f "$0"))

# Set ssh command used by rsync (we wil also call it directly later)
export RSYNC_RSH="ssh -o BatchMode=yes -o CheckHostIP=no -o HashKnownHosts=no"
# Store hostkey under a fixed name even if IP address is used, use local k_h
RSYNC_RSH="$RSYNC_RSH -o HostKeyAlias=odroid -o UserKnownHostsFile=known_hosts"

: ${RHOST:=odroid@mjmech-odroid}

if [[ "$1" == "stop" ]]; then
    $RSYNC_RSH $RHOST -t killall -v vserver.py
    exit $?
fi

RELEASE=$(lsb_release -sc)
if [[ "$RELEASE" != "precise" ]]; then
    # The program cannot even be imported on precise
    ./vserver/vserver.py --check
fi

# Assemble process
# Last component must be named 'vserver'
TMPDIR=/tmp/odroid-assemble/vserver
mkdir -p $TMPDIR
RSCMD=(rsync -a --exclude '*~' --exclude '*.pyc' --delete )

"${RSCMD[@]}" --exclude=legtool --exclude=install-packages.sh \
    --exclude=gbulb --exclude=real.cfg --exclude=vui_helpers.py \
    vserver $TMPDIR/..
"${RSCMD[@]}" ../legtool ../install-packages.sh ../gbulb ../real.cfg \
    vui_helpers.py $TMPDIR

if [[ "$1" == "assemble" ]]; then
    echo Assembled
    exit 0
fi

echo Uploading
rsync -a --delete $TMPDIR $RHOST:/tmp/
if [[ "$1" == "start" ]]; then
    echo Starting vserver in background
    $RSYNC_RSH $RHOST /tmp/vserver/getlogs.py --restart --wait 10 --last 0
    # If we failed, we will exit here.
    echo '>>> Success <<<'
elif [[ "$1" == "logs" ]]; then
    echo Connecting to $RHOST
    shift
    $RSYNC_RSH $RHOST /tmp/vserver/getlogs.py "$@"
elif [[ "$1" == "fg-start" ]]; then
    echo Starting vserver
    $RSYNC_RSH $RHOST -t exec /tmp/vserver/start.sh
elif [[ "$1" = "exec" ]]; then
    shift
    set -x
    $RSYNC_RSH $RHOST -Yt cd /tmp/vserver \&\& "$@"
fi
