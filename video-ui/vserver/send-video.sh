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

# settings below may be customized when running on desktop
: ${VMODE:=v}
: ${VDEVICE:=/dev/video0}
: ${GST_BIN:=/opt/gstreamer-1.4.5/bin/}

if [[ "${VMODE}" == "u" ]]; then
    # Use 'uvch264src' which lets us customize stream properties
    # (see http://www.oz9aec.net/index.php/gstreamer/487-using-the-logitech-c920-webcam-with-gstreamer-12)
    GSS="uvch264src device=${VDEVICE} name=src auto-start=true"
    #GSS+=" mode=mode-video post-previews=false "
    #GSS+=" iframe-period=1000"
    # Discard viewfinder image.
    GSS+=" src.vfsrc ! queue "
    GSS+=" ! video/x-raw,format=(string)YUY2,width=320,height=240,framerate=5/1"
    GSS+=" ! fakesink name=vfsrc-sink silent=true"
    # Send compressed video via queue
    GSS+=" src.vidsrc ! queue"
else
    # Use v4l2src which is more reliable but does not let us configure the
    # stream.
    GSS="v4l2src device=${VDEVICE}"
fi

# Set requested resultion and framerate
GSS+=" ! video/x-h264, width=1920, height=1080, framerate=30/1"

if [[ "$HOST" == "--test" ]]; then
    set -x
    # Test mode -- capture and print stats for 300 buffers
    #export GST_DEBUG=4
    exec ${GST_BIN}gst-launch-1.0 -v -e $GSS ! h264parse ! \
        identity error-after=300 ! fakesink name=h264data silent=false sync=false
fi

set -x
exec ${GST_BIN}gst-launch-1.0 -v -e rtpbin name=rtpbin \
    $GSS ! h264parse ! rtph264pay ! \
    identity name=camera-data silent=false ! \
    rtprtxqueue max-size-packets=1000 ! rtpbin.send_rtp_sink_0 \
    rtpbin.send_rtp_src_0 ! udpsink port=$PORT host=$HOST \
    rtpbin.send_rtcp_src_0 ! udpsink port=$PORT1 host=$HOST sync=false async=false \
    udpsrc name=rtcp_sink port=$PORT2 timeout=30000000000 ! rtpbin.recv_rtcp_sink_0
