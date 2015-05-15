#ifndef _VIDEO_CLIENT_H
#define _VIDEO_CLIENT_H

#include <gst/gst.h>
#include <glib.h>

struct MainAppSL;

typedef struct VideoClient {
  // Main pipeline element.
  GstElement* pipeline;

  // Pointer to MainAppSL object which will collect statistics.
  struct MainAppSL* main_app_sl;

  gchar* opt_input;
  gchar* opt_save_stream;
} VideoClient;

VideoClient* video_client_make();
void video_client_add_options(VideoClient*, GOptionGroup*);
void video_client_start(VideoClient*);

#endif
