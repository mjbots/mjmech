#include "assert.h"

#include <gst/gst.h>

#include "camera-recv.h"
#include "rtsp-server.h"


/*
To test playback:

gst-launch-1.0 rtspsrc location=rtsp://mjmech-odroid:8554/video latency=200 ! decodebin ! timeoverlay ! videoconvert ! xvimagesink
vlc --rtsp-caching=200  rtsp://127.0.0.1:8554/video
mpv --cache-secs=0.2 rtsp://127.0.0.1:8554/video

*/
int main (int argc, char *argv[])
{
  gst_init(&argc, &argv);

  // Exit on critical messages
  g_log_set_fatal_mask(NULL, G_LOG_LEVEL_ERROR | G_LOG_LEVEL_CRITICAL);

  GMainLoop* loop = g_main_loop_new(NULL, FALSE);

  CameraReceiver* receiver = camera_receiver_make();
  assert(receiver != NULL);

  // Create and register RTSP server
  RtspServer* server = rtsp_server_make();
  assert(server != NULL);

  receiver->rtsp_server = server;

  camera_receiver_start(receiver);
  rtsp_server_start(server);

  g_main_loop_run(loop);
  return 0;
}
