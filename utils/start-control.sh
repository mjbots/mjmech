#!/bin/bash
jsname=$(readlink -e \
        /dev/input/by-id/usb-Logitech_Logitech_Dual_Action-event-joystick)
if [[ "$jsname" == "" ]]; then
    echo Fatal: no joystick found
    exit 1
fi
mkdir -p logs
now=$(date +%F-%H%M%S)
set -x

# Use REMOTE env var to set remote IP
: ${REMOTE:=192.168.16.47}

./bazel-out/k8-opt/bin/mech/mw_command \
    --log logs/mwc-$now.klg \
    --opt.verbose 1 --joystick $jsname \
    -t video_display.stats --display.stats_interval_s 30 \
    --target $REMOTE \
    --video_link.dest $REMOTE \
    --remote_debug.port 9999 \
    --video_link.source 192.168.16.42:2180 \
    --opt.max_translate_x_mm_s 100 --opt.max_translate_y_mm_s 100 \
    --opt.max_rotate_deg_s=20 \
    "$@"
