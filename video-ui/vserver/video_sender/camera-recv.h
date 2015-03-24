#ifndef _CAMERA_RECV_H
#define _CAMERA_RECV_H

#include <gst/gst.h>
#include <glib.h>

struct RtspServer;

typedef struct CameraReceiver {
  // Main pipeline element.
  GstElement* pipeline;
  // Statistics to print.
  GHashTable* stats;

  // Pointer to RtspServer object. Should only be modified
  // before 'start'. May be left at NULL.
  struct RtspServer* rtsp_server;

  // Options. Should only be modified before 'start'.
  gchar* opt_device;
  gchar* opt_save_h264;
  gboolean opt_dumb_camera;

} CameraReceiver;

// Allocate object. Does not do any gst calls.
CameraReceiver* camera_receiver_make();

// Add command-line for this object.
void camera_receiver_add_options(CameraReceiver*, GOptionGroup*);

// Increment given value in stats by that much
void camera_receiver_add_stat(CameraReceiver*, const char* name, int inc);

void camera_receiver_start(CameraReceiver*);

#endif
