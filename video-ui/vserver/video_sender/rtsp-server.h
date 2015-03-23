#ifndef _RTSP_SERVER_H
#define _RTSP_SERVER_H

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>

typedef struct RtspServer {
  // Main server object.
  GstRTSPServer* server;
  // The factory for default URL
  GstRTSPMediaFactory* factory;
  // Constructed media. When this is non-null, this ensures factory
  // is alive even if there are no clients.
  GstRTSPMedia* factory_media;

  // Mutex for all appsrc_ variables, as they may be changed from various threads.
  GMutex appsrc_mutex;

  // Current H264 appsource, or NULL if no active connection.
  // Set by rtsp-server.
  // Make sure to AddRef if using outside of the mutex.
  // set to NULL when there is no active connections.
  GstAppSrc* appsrc_h264;

  // Caps to for appsource. Set by external apps.
  GstCaps* appsrc_h264_caps;

  int error_count;
} RtspServer;

RtspServer* rtsp_server_make();

void rtsp_server_start(RtspServer*);

#endif
