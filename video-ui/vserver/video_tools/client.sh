#!/bin/bash


# Build and run the tool
# (gcc is so fast we can just build it on every app start)
mydir=$(dirname $(readlink -f "$0"))

export LD_LIBRARY_PATH=/opt/gstreamer-1.4.5/lib
export PKG_CONFIG_PATH=$LD_LIBRARY_PATH/pkgconfig

set -e
FLAGS=$(pkg-config --cflags --libs \
    gstreamer-rtsp-1.0 gstreamer-sdp-1.0 \
    gstreamer-1.0 gstreamer-base-1.0 gstreamer-app-1.0 \
    glib-2.0 gobject-2.0)

(cd $mydir &&
    set -x &&
    gcc -Wall -Werror -g -o /tmp/video_client.exe \
        cli-main.c video-client.c main-app-sl.c $FLAGS
)

if [[ "$1" == "--test" ]]; then
    echo video_client compiled succesfully
    exit 0
fi

# Set reasonable debug values
: ${GST_DEBUG:=3,v4l2:0}

exec /tmp/video_client.exe "$@"
