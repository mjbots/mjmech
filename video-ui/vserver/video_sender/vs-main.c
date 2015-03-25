#include <assert.h>
#include <stdlib.h>

#include "main-app-sl.h"
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
  MainAppSL* mainapp = main_app_sl_make();
  CameraReceiver* receiver = camera_receiver_make();
  RtspServer* server = rtsp_server_make();

  // Use one shared group -- we do not have enough options to require
  // a group per .c file.
  camera_receiver_add_options(receiver, mainapp->main_group);
  rtsp_server_add_options(server, mainapp->main_group);

  receiver->rtsp_server = server;
  receiver->main_app_sl = mainapp;

  main_app_sl_start(mainapp, argc, argv);
  camera_receiver_start(receiver);
  rtsp_server_start(server);

  main_app_sl_loop(mainapp);

  // Flush the buffers
  camera_receiver_stop(receiver);
  // We do not care about properly stopping RTSP server

  return mainapp->exit_code;
}
