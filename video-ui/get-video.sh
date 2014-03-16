gst-launch-1.0 -v udpsrc port=13357 ! \
   "application/x-rtp,payload=96,encoding-name=H264" ! \
   rtph264depay ! h264parse ! avdec_h264 ! xvimagesink
