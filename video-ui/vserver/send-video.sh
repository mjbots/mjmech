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
exec gst-launch-1.0 -v -e rtpbin name=rtpbin v4l2src device=/dev/video0 ! \
    video/x-h264, width=1920, height=1080, framerate=30/1 ! h264parse ! rtph264pay ! \
    udpsink port=$PORT host=$HOST
