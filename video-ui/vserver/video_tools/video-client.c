#include "video-client.h"

#include <assert.h>
#include <string.h>

#include "main-app-sl.h"


VideoClient* video_client_make() {
  VideoClient* this = g_malloc0(sizeof(VideoClient));
  return this;

}

void video_client_add_options(VideoClient* this, GOptionGroup* group) {
  GOptionEntry entries[] = {
    { "input", 'I', 0, G_OPTION_ARG_STRING, &this->opt_input,
      "Input (RTSP url, or file, or 'TEST')", "URL" },
    { NULL },
  };

  g_option_group_add_entries(group, entries);
}

static gchar* make_launch_cmd(VideoClient* this) {
  GString* result = g_string_new(NULL);

  if  (strcmp(this->opt_input, "TEST") == 0) {
        g_string_append(result, "videotestsrc is-live=1 pattern=ball ");
  } else {
    g_string_append_printf(
        result, "rtspsrc location=%s latency=200", this->opt_input);
  }

  g_string_append(
      result,
      " ! decodebin ! timeoverlay ! videoconvert ! xvimagesink name=imagesink");

  return g_string_free(result, FALSE); // Free the struct, return the contents.
}

static gboolean bus_message(GstBus     *bus,
                        GstMessage *message,
                        gpointer    user_data) {
  VideoClient* this = (VideoClient*)user_data;
  gboolean print_message = TRUE;
  switch (GST_MESSAGE_TYPE (message)) {
    case GST_MESSAGE_EOS: {
      g_message("EOS on pipeline");
      main_app_sl_exit(this->main_app_sl);
      return TRUE;
    }
    case GST_MESSAGE_ERROR: {
      GError *err = NULL;
      gchar *dbg = NULL;
      gst_message_parse_error(message, &err, &dbg);
      if (err) {
        g_warning("Pipeline ERROR: %s\nDebug details: %s",
                  err->message, dbg ? dbg : "(NONE)" );
        g_error_free(err);
        main_app_sl_exit(this->main_app_sl);
      }
      if (dbg) { g_free(dbg); }
      break;
    }
    case GST_MESSAGE_STATE_CHANGED:
    case GST_MESSAGE_STREAM_STATUS:
    case GST_MESSAGE_TAG:
    case GST_MESSAGE_NEW_CLOCK: {
      // Ignore
      print_message = FALSE;
      break;
    }
    default: { break; };
  }
  if (print_message) {
    const GstStructure* mstruct = gst_message_get_structure(message);
    char* struct_info =
        mstruct ? gst_structure_to_string(mstruct) : g_strdup("no-struct");
    g_message("video-client message '%s' from '%s': %s",
              GST_MESSAGE_TYPE_NAME(message),
              GST_MESSAGE_SRC_NAME(message),
              struct_info);
    g_free(struct_info);
  }
  return TRUE;
}

void video_client_start(VideoClient* this) {
  if (!this->opt_input || !*this->opt_input) {
    g_warning("Cannot create client: no input specified");
    main_app_sl_exit(this->main_app_sl);
    return;
  }

  char* launch_cmd = make_launch_cmd(this);
  g_message("Creating client pipeline: gst-launch-1.0 %s", launch_cmd);
  // Start the main pipeline which reads the video
  GError* error = NULL;
  this->pipeline = gst_parse_launch(launch_cmd, &error);
  g_free(launch_cmd);
  if ((!this->pipeline) || error) {
    g_warning("Client LAUNCH_CMD error %d: %s", error->code, error->message);
    main_app_sl_exit(this->main_app_sl);
    return;
  }
  assert(error == NULL);

  main_app_sl_add_stat(this->main_app_sl, "started", 1);

  GstBus* bus = gst_element_get_bus(this->pipeline);
  gst_bus_add_watch(bus, bus_message, this);
  gst_object_unref(bus);

  // start the pipeline
  gst_element_set_state(this->pipeline, GST_STATE_PLAYING);
}
