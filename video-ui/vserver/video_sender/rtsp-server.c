#include "rtsp-server.h"

#include <assert.h>
#include <stdlib.h>

static const char LAUNCH_CMD[] = "( "
  "appsrc name=h264-input max-bytes=1024000 block=false "
  "   do-timestamp=true is-live=true format=time min-latency=1 max-latency=1"
  " ! rtph264pay name=pay0 pt=96 "
  ")";

static const char LAUNCH_CMD_1[] = "( "
  "videotestsrc ! x264enc "
  " ! rtph264pay name=pay0 pt=96 "
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

void media_bus_message(GstBus     *bus,
                              GstMessage *message,
                              gpointer    user_data) {
  switch (GST_MESSAGE_TYPE (message)) {
  case GST_MESSAGE_ERROR: {
    GError *err = NULL;
    gchar *dbg = NULL;
    gst_message_parse_error(message, &err, &dbg);
    if (err) {
      g_error("RTSP Pipeline ERROR: %s", err->message);
      g_error_free(err);
    }
    if (dbg) {
      g_message("Debug details: %s", dbg);
      g_free(dbg);
    }
    break;
  }
    /*
  case GST_MESSAGE_STATE_CHANGED:
  case GST_MESSAGE_STREAM_STATUS:
  case GST_MESSAGE_TAG:
  case GST_MESSAGE_NEW_CLOCK: {
    // Ignore
    break;
  }
    */
  default: {
    const GstStructure* mstruct = gst_message_get_structure(message);
    char* struct_info =
      mstruct ? gst_structure_to_string(mstruct) : g_strdup("no-struct");
    g_message("RTSP bus Message '%s' from '%s': %s",
              GST_MESSAGE_TYPE_NAME(message),
              GST_MESSAGE_SRC_NAME(message),
              struct_info);
    g_free(struct_info);
    break;
  }
  }
}

static void media_unprepared(GstRTSPMedia *gstrtspmedia,
                             RtspServer          *this) {
  g_message("Tearing down video source, there were %d errors",
            this->error_count);
  g_mutex_lock(&this->appsrc_mutex);
  if (this->appsrc_h264) {
    gst_object_unref(this->appsrc_h264);
    this->appsrc_h264 = NULL;
  }
  g_mutex_unlock(&this->appsrc_mutex);
}

static void media_new_state(GstRTSPMedia *gstrtspmedia, gint state,
                            gpointer      user_data) {
  // TODO: if media did not go to PLYAING state after a while, raise an error.
  g_message("Video source in state %d", state);
}


static int kMaxErrors = 20;
static void appsrc_enough_data(GstAppSrc *src, RtspServer *this) {
  if (this->error_count < kMaxErrors) {
    char* name = gst_element_get_name(src);
    g_message("RTSP appsource '%s' says: enough, dropping data", name);
    g_free(name);
  };
  this->error_count++;
  if (this->error_count == kMaxErrors) {
    g_message("Further errors suppressed");
  }
}

static void media_configure(GstRTSPMediaFactory *sender,
                            GstRTSPMedia        *media,
                            RtspServer          *this) {
  g_message("Starting up video source");
  this->error_count = 0;
  g_signal_connect(media, "unprepared", (GCallback)media_unprepared, this);
  g_signal_connect(media, "new-state", G_CALLBACK(media_new_state), this);

  GstElement* main_bin = gst_rtsp_media_get_element(media);

  // Watch bus messages
  GstBus* bus = gst_element_get_bus(main_bin);
  g_signal_connect(bus, "message", G_CALLBACK(media_bus_message), this);
  //gst_bus_add_signal_watch(bus);
  gst_object_unref(GST_OBJECT(bus));

  // Find out appsink and store it
  g_mutex_lock(&this->appsrc_mutex);
  assert(this->appsrc_h264 == NULL);
  this->appsrc_h264 =
    GST_APP_SRC(gst_bin_get_by_name_recurse_up(GST_BIN(main_bin),
                                               "h264-input"));
  if (this->appsrc_h264) {
    if (this->appsrc_h264_caps) {
      // Function takes a copy of caps, so original reference is still in this
      // obect.
      gst_app_src_set_caps(this->appsrc_h264, this->appsrc_h264_caps);
    }
    g_signal_connect(this->appsrc_h264, "enough-data",
                     G_CALLBACK(appsrc_enough_data), this);
  } else {
    g_warning("Could not find H264 appsource in RTSP pipeline");
  }
  g_mutex_unlock(&this->appsrc_mutex);

  gst_object_unref(main_bin);
}


RtspServer* rtsp_server_make() {
  RtspServer* this = calloc(1, sizeof(RtspServer));

  g_mutex_init(&this->appsrc_mutex);
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
  return this;
}

void rtsp_server_start(RtspServer* this) {
  gst_rtsp_server_attach(this->server, NULL);
  g_message("stream ready at rtsp://127.0.0.1:%d/video",
            gst_rtsp_server_get_bound_port(this->server));
}
