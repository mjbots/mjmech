#include "camera-recv.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include <gst/app/gstappsink.h>

#include "rtsp-server.h"

// TODO: find right camera by glob
//const char UVC_CAMERA_GLOB[] =
//    "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_*-video-index0";

const char DEFAULT_DEVICE[] = "/dev/video0";

// Caps for locally decoded image
// (this is what is requested from uvch264src.vfsrc, and there are
// limits on minimal framerate and resolution)
const char DECODED_IMAGE_CAPS[] =
    "video/x-raw,format=YUY2,width=640,height=480,framerate=30/1";
// H264 caps when useing on-camera encoder
const char HW_H264_CAPS[] =
    "video/x-h264,width=1920,height=1080,framerate=30/1";
// H264 caps when using local encoder. Framerate/size is all derived.
const char SW_H264_CAPS[] =
    "video/x-h264";


static char* make_launch_cmd(CameraReceiver* this) {
  GString* result = g_string_new(NULL);
  const char* dev = this->opt_device ? this->opt_device : DEFAULT_DEVICE;
  gboolean is_test = (strcmp(dev, "TEST") == 0);
  gboolean is_dumb = is_test || this->opt_dumb_camera;

  // TODO: the pipeline is not very optimal when is_dumb is True -- too many
  // videocoverts. Whatever, it is for development anyway.

  // TODO: the pipeline probably has too many 'queue' elements -- each one is
  // a thread. However, the pipeline will silently fail to start if there are
  // too few.

  // make unencoded image endpoint
  if (is_test) {
    g_string_append(result, "videotestsrc is-live=1 pattern=ball ");
  } else if (is_dumb) {
    g_string_append_printf(
        result, "v4l2src name=src device=%s ! videoconvert ",
        dev);
  } else {
    g_string_append_printf(
        result, "uvch264src device=%s name=src auto-start=true "
        "   iframe-period=1000 "
        "src.vfsrc ", dev);
  }
  g_string_append_printf(result, " ! %s ", DECODED_IMAGE_CAPS);
  if (is_dumb) {
    g_string_append(result, " ! tee name=dec-tee ");
  }

  // consume unencoded image data
  g_string_append(result,
                  " ! queue ! appsink name=raw-sink max-buffers=1 drop=true ");

  // make h264 image endpoint
  if (is_dumb) {
    g_string_append_printf(
        result, " dec-tee. ! videoconvert ! queue "
        " ! x264enc tune=zerolatency ! %s ",
        SW_H264_CAPS);
  } else {
    g_string_append_printf(
        result, " src.vidsrc ! %s ! h264parse ",
        HW_H264_CAPS);
  }

  // consume h264 data
  if (this->opt_save_h264 && *this->opt_save_h264) {
    char* muxer = NULL;
    g_message("Will save H264 to file %s", this->opt_save_h264);
    if (g_str_has_suffix(this->opt_save_h264, ".mkv")) {
      // MKV has useful metadata (like stream start time), but buffers
      // in large chunks.
      muxer = "matroskamux streamable=true";
    } else if (g_str_has_suffix(this->opt_save_h264, ".mp4")) {
      // mp4 does not buffer much.
      muxer = "mp4mux";
    } else if (g_str_has_suffix(this->opt_save_h264, ".avi")) {
      muxer = "avimux";
    } else {
      g_warning("Unknown h264 savefile extension, assuming MP4 format");
      muxer = "mp4mux";
    }
    g_string_append_printf(
        result, " ! tee name=h264-tee ! queue"
        " ! %s ! filesink name=h264writer location=%s "
        " h264-tee.", muxer, this->opt_save_h264);
  }
  g_string_append(
      result,
      "! queue ! appsink name=h264-sink max-buffers=120 drop=false ");


  return g_string_free(result, FALSE); // Free the struct, return the contents.
}

void camera_receiver_add_stat(CameraReceiver* this, const char* name, int inc) {
  int* val = g_hash_table_lookup(this->stats, name);
  if (!val) {
    val = g_malloc0(sizeof(int));
    g_hash_table_insert(this->stats, g_strdup(name), val);
  }
  *val += inc;
}

static gboolean print_stats(void* user_arg) {
  CameraReceiver* this = (CameraReceiver*)user_arg;
  GList* keys = g_hash_table_get_keys(this->stats);
  keys = g_list_sort(keys, (GCompareFunc)strcmp);

  GString* message = g_string_new("");

  GList* key;
  for (key=keys; key; key=key->next) {
    int* val = (int*)g_hash_table_lookup(this->stats, key->data);
    g_string_append_printf(message, "%s=%d; ", (char*)key->data, *val);
  }
  g_list_free(keys);
  g_hash_table_remove_all(this->stats);

  g_message("Stats: %s", message->str);
  g_string_free(message, TRUE);
  return TRUE;
}

static GstFlowReturn raw_sink_new_sample(GstElement* sink,
                                         CameraReceiver* this) {
  GstSample* sample = NULL;
  g_signal_emit_by_name(sink, "pull-sample", &sample);
  if (!sample) {
    g_warning("raw sink is out of samples");
    return GST_FLOW_OK;
  }

  //GstCaps* caps = gst_sample_get_caps(sample);
  //GstBuffer* buf = gst_sample_get_buffer(sample);

  //g_message("raw sink got sample");
  camera_receiver_add_stat(this, "raw-sample", 1);
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

static void raw_sink_configure(GstElement* parent_bin, gpointer user_data) {
  GstAppSink* raw_sink = GST_APP_SINK(
      gst_bin_get_by_name_recurse_up(GST_BIN(parent_bin), "raw-sink"));
  if (raw_sink == NULL) {
    g_warning("Could not find raw-sink");
    // assert(false);
    return;
  };

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
                   G_CALLBACK(raw_sink_new_sample), user_data);

  //g_util_set_object_arg(G_OBJECT(raw_sink),
  gst_object_unref(raw_sink);
}

static GstFlowReturn h264_sink_new_sample(GstElement* sink,
                                          CameraReceiver* this) {
  GstSample* sample = NULL;
  g_signal_emit_by_name(sink, "pull-sample", &sample);
  if (!sample) {
    g_warning("h264 sink is out of samples");
    return GST_FLOW_OK;
  }

  if (rtsp_server_push_h264_sample(this->rtsp_server, sample)) {
    camera_receiver_add_stat(this, "h264-buff-sent", 1);
  } else {
    camera_receiver_add_stat(this, "h264-buff-ignored", 1);
  }
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

static void h264_sink_configure(GstElement* parent_bin, gpointer user_data) {
  GstAppSink* h264_sink = GST_APP_SINK(
      gst_bin_get_by_name_recurse_up(GST_BIN(parent_bin), "h264-sink"));
  assert(h264_sink != NULL);
  gst_app_sink_set_emit_signals(h264_sink, TRUE);
  g_signal_connect(h264_sink, "new-sample",
                   G_CALLBACK(h264_sink_new_sample), user_data);
  gst_object_unref(h264_sink);
}

static void bus_message(GstBus     *bus,
                        GstMessage *message,
                        gpointer    user_data) {
  switch (GST_MESSAGE_TYPE (message)) {
  case GST_MESSAGE_EOS: {
    g_message("EOS on pipeline");
    exit(1);
    break;
  }
  case GST_MESSAGE_ERROR: {
    GError *err = NULL;
    gchar *dbg = NULL;
    gst_message_parse_error(message, &err, &dbg);
    if (err) {
      g_error("Pipeline ERROR: %s\nDebug details: %s",
              err->message, dbg ? dbg : "(NONE)" );
      g_error_free(err);
    }
    if (dbg) { g_free(dbg); }
    break;
  }
  case GST_MESSAGE_STATE_CHANGED:
  case GST_MESSAGE_STREAM_STATUS:
  case GST_MESSAGE_TAG:
  case GST_MESSAGE_NEW_CLOCK: {
    // Ignore
    break;
  }
  default: {
    const GstStructure* mstruct = gst_message_get_structure(message);
    char* struct_info =
      mstruct ? gst_structure_to_string(mstruct) : g_strdup("no-struct");
    g_message("Message '%s' from '%s': %s",
              GST_MESSAGE_TYPE_NAME(message),
              GST_MESSAGE_SRC_NAME(message),
              struct_info);
    g_free(struct_info);
    break;
  }
  }
}

CameraReceiver* camera_receiver_make() {
  CameraReceiver* this = g_malloc0(sizeof(CameraReceiver));

  this->stats = g_hash_table_new_full(g_str_hash, g_str_equal, g_free, g_free);
  return this;
}

void camera_receiver_add_options(CameraReceiver* this, GOptionGroup* group) {
  GOptionEntry entries[] = {
    { "device", 'd', 0, G_OPTION_ARG_FILENAME, &this->opt_device,
      "Video device file to use, 'TEST' for test source", DEFAULT_DEVICE },
    { "save-h264", 's', 0, G_OPTION_ARG_FILENAME, &this->opt_save_h264,
      "Record received H264 stream to this file", "file.MKV|MP4|AVI" },
    { "dumb-camera", 0, 0, G_OPTION_ARG_NONE, &this->opt_dumb_camera,
      "Assume camera does not do H264 stream, encode on CPU" },
    { NULL },
  };

  g_option_group_add_entries(group, entries);
}


void camera_receiver_start(CameraReceiver* this) {

  char* launch_cmd = make_launch_cmd(this);
  g_message("Creating pipeline: gst-launch-1.0 %s", launch_cmd);
  // Start the main pipeline which reads the video
  GError* error = NULL;
  this->pipeline = gst_parse_launch(launch_cmd, &error);
  g_free(launch_cmd);
  if (!this->pipeline || error) {
    g_message("LAUNCH_CMD error %d: %s", error->code, error->message);
    exit(1);
  }
  assert(error == NULL);

  GstBus* bus = gst_element_get_bus(this->pipeline);
  g_signal_connect(bus, "message", G_CALLBACK(bus_message), this);
  gst_bus_add_signal_watch(bus);
  gst_object_unref(GST_OBJECT(bus));

  raw_sink_configure(this->pipeline, this);
  h264_sink_configure(this->pipeline, this);

  camera_receiver_add_stat(this, "started", 1);

  // start the pipeline
  gst_element_set_state(this->pipeline, GST_STATE_PLAYING);
  // start stats timer
  g_timeout_add(5000, print_stats, this);
}
