#ifndef _CAMERA_RECV_H
#define _CAMERA_RECV_H

#include <gst/gst.h>
#include <glib.h>

typedef struct CameraReceiver {
  GstElement* pipeline;
  // Statistics
  GHashTable* stats;
} CameraReceiver;

CameraReceiver* camera_receiver_make();

// Increment given value in stats by that much 
void camera_receiver_add_stat(CameraReceiver*, const char* name, int inc);

void camera_receiver_start(CameraReceiver*);

#endif
