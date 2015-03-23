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
  // (via push_h264_sample function)
  GMutex appsrc_mutex;

  // Current H264 appsource, or NULL if no active connection.
  // Make sure to AddRef if using outside of the mutex.
  // set to NULL when there is no active connections.
  GstAppSrc* appsrc_h264;
  // Caps to be applied to appsource
  GstCaps* appsrc_h264_caps;
  // Buffers since start of the session
  int appsrc_h264_buffers;
  // True if offset fields are valid.
  gboolean appsrc_offset_valid;
  GstClockTime appsrc_offset_dts;
  GstClockTime appsrc_offset_pts;
  gboolean appsrc_needs_iframe;

  // Number of printed errors. Reset when no connection is active.
  int error_count;
} RtspServer;

RtspServer* rtsp_server_make();

// Push a foreign sample to h264 appsource.
// Returns TRUE if sample was pushed, FALSE if push failed.
// Does not take ownership of sample.
// server pointer may be NULL.
gboolean rtsp_server_push_h264_sample(RtspServer*, GstSample*);

void rtsp_server_start(RtspServer*);

#endif
