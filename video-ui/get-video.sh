#!/bin/bash
PORT=13357
PORT1=$(expr $PORT + 1)
PORT2=$(expr $PORT + 2)

HOST="$1"
if [[ "$HOST" == "" ]]; then
    echo Error: expected data sender is not specified.
    exit 1
fi


if [[ "$DISPLAY" == "" ]]; then
    echo No X, using text output
    # this will print a line for each buffer received
    SINK='fakesink silent=false'
else
    SINK=xvimagesink
fi

set -x -e
gst-launch-1.0 -v rtpbin name=rtpbin do-retransmission=true \
    udpsrc caps="application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264" \
      port=$PORT ! rtpbin.recv_rtp_sink_0 \
    rtpbin. ! rtph264depay ! h264parse ! avdec_h264 ! \
    $SINK \
    udpsrc port=$PORT1 ! rtpbin.recv_rtcp_sink_0 \
    rtpbin.send_rtcp_src_0 ! udpsink host=$HOST port=$PORT2 sync=false async=false

#
