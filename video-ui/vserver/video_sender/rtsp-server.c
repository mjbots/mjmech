#include "rtsp-server.h"

#include <assert.h>
#include <stdlib.h>

#include <gst/app/gstappsink.h>

const char LAUNCH_CMD_1[] = "( "
    "videotestsrc is-live=1 "
    " ! x264enc ! rtph264pay name=pay0 pt=96 "
    "videotestsrc is-live=1 "
    " ! appsink name=raw-sink max-buffers=1 drop=true "
    ")";

const char LAUNCH_CMD[] = "( "
    "uvch264src device=/dev/video0 name=src auto-start=true "
    //"src.vfsrc ! queue "
    //" ! video/x-raw,format=(string)YUY2,width=320,height=240,framerate=5/1"
    //" ! appsink name=raw-sink max-buffers=1 drop=true "
    //" ! fakesink name=vfsrc-sink silent=true "
    "src.vidsrc ! video/x-h264, width=1920, height=1080, framerate=30/1 "
    " ! queue ! rtph264pay name=pay0 pt=96 "
    ")";

const char LAUNCH_CMD_3[] = "( "
    "v4l2src device=/dev/video0 "
    " ! video/x-h264, width=1920, height=1080, framerate=30/1 "
    " ! queue ! rtph264pay name=pay0 pt=96 "
    ")";

static void client_closed(GstRTSPClient *gstrtspclient,
                          gpointer       user_data) {
  g_message("client %p disconnected", gstrtspclient);
}


static void client_connected(GstRTSPServer *gstrtspserver,
                             GstRTSPClient *client,
                             gpointer       user_data) {
  GstRTSPConnection* conn = gst_rtsp_client_get_connection(client);
  g_message("client %p connected from %s",
          client, gst_rtsp_connection_get_ip(conn));
  g_signal_connect(client, "closed", (GCallback)client_closed,
                   NULL);
}

static void media_unprepared(GstRTSPMedia *gstrtspmedia,
                      gpointer      user_data) {
  g_message("Tearing down video source");
}

static void media_new_state(GstRTSPMedia *gstrtspmedia, gint state,
                            gpointer      user_data) {
  // TODO: if media did not go to PLYAING state after a while, raise an error.
  g_message("Video source in state %d", state);
}


GstFlowReturn raw_sink_new_sample(GstElement* sink) {
  GstSample* sample = NULL;
  g_signal_emit_by_name(sink, "pull-sample", &sample);
  if (!sample) {
    g_warning("raw sink is out of samples");
    return GST_FLOW_OK;
  }

  //GstCaps* caps = gst_sample_get_caps(sample);
  //GstBuffer* buf = gst_sample_get_buffer(sample);

  g_message("raw sink got sample");
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

void media_configure(GstRTSPMediaFactory *sender,
                     GstRTSPMedia        *media,
                     gpointer             user_data) {
  g_message("Starting up video source");
  g_signal_connect(media, "unprepared", (GCallback)media_unprepared,
                   user_data);
  g_signal_connect(media, "new-state", G_CALLBACK(media_new_state), user_data);
  // Find out appsink and hook to it
  GstElement* main_bin = gst_rtsp_media_get_element(media);

  GstAppSink* raw_sink = GST_APP_SINK(
      gst_bin_get_by_name_recurse_up(GST_BIN(main_bin), "raw-sink"));
  if (raw_sink == NULL) {
    g_warning("Could not find raw-sink");
    // assert(false);
  } else {
    assert(raw_sink != NULL);
    // TODO mafanasyev: call gst_app_sink_set_caps(raw_sink, ...)
    // or add 'caps' property to raw_sink

    // We emit a signal every time we have a frame; if the buffer
    // is not pulled by the next frame, we discard it.
    // Note that the alternative is to have a separate thread which
    // just pulls all the time (pull function will block if there
    // is no data)
    gst_app_sink_set_emit_signals(raw_sink, TRUE);
    gst_app_sink_set_max_buffers(raw_sink, 1);
    gst_app_sink_set_drop(raw_sink, TRUE);
    g_signal_connect(raw_sink, "new-sample",
                     G_CALLBACK(raw_sink_new_sample), NULL);

    //g_util_set_object_arg(G_OBJECT(raw_sink),
    gst_object_unref(raw_sink);
  }

  gst_object_unref(main_bin);
}

gboolean start_media_factory(void* user) {
  g_message("Starting media factory");
  /*
  RtspServer* this = (RtspServer*)user;

  GstRTSPUrl* url = NULL;
  GstRTSPResult ok = gst_rtsp_url_parse("rtsp://localhost/", &url);
  assert(ok == GST_RTSP_OK);
  this->factory_media = gst_rtsp_media_factory_construct(
      this->factory, url);
  gst_rtsp_url_free(url);
  */
  return FALSE;   // Do not call again
}

RtspServer* make_rtsp_server() {
  RtspServer* this = calloc(1, sizeof(RtspServer));

  this->server = gst_rtsp_server_new();

  GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(this->server);

  this->factory = gst_rtsp_media_factory_new();
  gst_rtsp_media_factory_set_launch(this->factory, LAUNCH_CMD);
  // One connection for all
  gst_rtsp_media_factory_set_shared(this->factory, TRUE);
  // Ignore pause requests from clients
  gst_rtsp_media_factory_set_suspend_mode(
      this->factory, GST_RTSP_SUSPEND_MODE_NONE);
  // TODO mafanasyev: set "latency"? it is 500+ sec by default
  g_signal_connect(
      this->factory, "media-configure", (GCallback) media_configure, this);

  gst_rtsp_mount_points_add_factory(mounts, "/video", this->factory);
  g_object_unref(mounts);

  g_signal_connect(this->server, "client-connected",
                   (GCallback)client_connected, this);

  // Once server is ready, start the stream
  g_idle_add(start_media_factory, this);

  gst_rtsp_server_attach(this->server, NULL);
  g_message("stream ready at rtsp://127.0.0.1:%d/video",
            gst_rtsp_server_get_bound_port(this->server));

  return this;
}
