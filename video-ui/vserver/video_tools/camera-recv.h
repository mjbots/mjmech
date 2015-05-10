#ifndef _CAMERA_RECV_H
#define _CAMERA_RECV_H

#include <gst/gst.h>
#include <glib.h>

struct RtspServer;
struct MainAppSL;

typedef struct CameraReceiver {
  // Main pipeline element.
  GstElement* pipeline;

  // Pointer to RtspServer object. Should only be modified
  // before 'start'. May be left at NULL.
  struct RtspServer* rtsp_server;

  // Pointer to MainAppSL object which will collect statistics.
  struct MainAppSL* main_app_sl;

  // Options. Should only be modified before 'start'.
  gchar* opt_device;
  gchar* opt_save_h264;
  gboolean opt_dumb_camera;

} CameraReceiver;

// Allocate object. Does not do any gst calls.
CameraReceiver* camera_receiver_make();

// Add command-line for this object.
void camera_receiver_add_options(CameraReceiver*, GOptionGroup*);

void camera_receiver_start(CameraReceiver*);
void camera_receiver_stop(CameraReceiver*);

#endif
