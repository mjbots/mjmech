#ifndef _RTSP_SERVER_H
#define _RTSP_SERVER_H

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>

typedef struct RtspServer {
  // Main server object.
  GstRTSPServer* server;
  // The factory for default URL
  GstRTSPMediaFactory* factory;
  // Constructed media. When this is non-null, this ensures factory
  // is alive even if there are no clients.
  GstRTSPMedia* factory_media;

} RtspServer;

RtspServer* make_rtsp_server();

#endif
