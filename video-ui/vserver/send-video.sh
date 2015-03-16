#!/bin/bash
HOST=$1
PORT=$2
if [ "$HOST" == "" ]; then
        read HOST ignored < <(echo $SSH_CLIENT)
        echo Auto-select host from ssh: $HOST
fi
if [ "$PORT" == "" ]; then
    PORT=13357
    echo Assuming port $PORT
fi

if [ "$HOST" == "" ]; then
        echo Error: host unset
        exit 1
fi

#debug
rm -rf gst-debug
mkdir -p gst-debug
#export GST_DEBUG_DUMP_DOT_DIR=gst-debug

# +0=RTP to player; +1=RTCP to player; +2=RTCP from player
PORT1=$(expr $PORT + 1)
PORT2=$(expr $PORT + 2)

# rtcp_sink has timeout property so we would get warnings if rtcp messages are
# not being received

# sources:
# (see http://www.oz9aec.net/index.php/gstreamer/487-using-the-logitech-c920-webcam-with-gstreamer-12)
#GSOURCE="uvch264src device=/dev/video0 iframe-period=3000 name=src"
#GSOURCE="$GSOURCE auto-start=true  src.vfsrc ! fakesink src.vidsrc"

GSOURCE="v4l2src device=/dev/video0"

set -x
exec gst-launch-1.0 -v -e rtpbin name=rtpbin \
    $GSOURCE ! video/x-h264, width=1920, height=1080, framerate=30/1 ! \
    h264parse ! rtph264pay ! \
    identity name=camera-data silent=false ! \
    rtprtxqueue max-size-packets=1000 ! rtpbin.send_rtp_sink_0 \
    rtpbin.send_rtp_src_0 ! udpsink port=$PORT host=$HOST \
    rtpbin.send_rtcp_src_0 ! udpsink port=$PORT1 host=$HOST sync=false async=false \
    udpsrc name=rtcp_sink port=$PORT2 timeout=30000000000 ! rtpbin.recv_rtcp_sink_0
