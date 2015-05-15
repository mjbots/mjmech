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
    { "save", 's', 0, G_OPTION_ARG_FILENAME, &this->opt_save_stream,
      "Record received stream to this file", "file.MKV" },
    { NULL },
  };

  g_option_group_add_entries(group, entries);
}

static gchar* make_launch_cmd(VideoClient* this) {
  GString* result = g_string_new(NULL);

  if  (strcmp(this->opt_input, "TEST") == 0) {
    g_string_append(
       result, 
       "videotestsrc is-live=1 pattern=ball ! "
       //"video/x-raw,format=YUY2,width=1920,height=1080,framerate=15/1"
       "video/x-raw,format=YUY2,width=640,height=480,framerate=15/1"
       );
  } else {
    g_string_append_printf(
        result, "rtspsrc location=%s latency=200", this->opt_input);
  }

  
  if (this->opt_save_stream && *this->opt_save_stream) {
    g_string_append_printf(
        result,
        " ! tee name=saver"
        " ! queue name=queue-save"
        " ! matroskamux"
        " ! filesink location=%s"
        " saver.",
        this->opt_save_stream);
  }

  g_string_append(result, " ! decodebin");

  // The pipeline stops when there is no connection, or when the frames are lost
  // TODO: insert code which generates dummy frames when there are none.
  g_string_append(
        result, 
        " ! videoconvert"
        " ! timeoverlay shaded_background=True font_desc=8 valignment=bottom halignment=right"
        " ! rsvgoverlay name=info_overlay fit_to_frame=True"
        " ! videoconvert name=presink"
        " ! xvimagesink name=imagesink");

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

static GstPadProbeReturn imagesink_event(GstPad* pad, GstPadProbeInfo* info, 
                                         void* user_data) {
  VideoClient* this = (VideoClient*)user_data;
  int stype = info->type &~ (GST_PAD_PROBE_TYPE_PUSH | GST_PAD_PROBE_TYPE_PULL);
  if (stype != GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM &&
      stype != GST_PAD_PROBE_TYPE_EVENT_UPSTREAM) {
    g_message("imagesink event of unknown type 0x%x", info->type);
    return GST_PAD_PROBE_OK;
  }
  GstEvent* event = gst_pad_probe_info_get_event(info);

  if (strcmp(GST_EVENT_TYPE_NAME(event), "qos") == 0) {
    GstQOSType qos_type;
    gst_event_parse_qos(event, &qos_type, NULL, NULL, NULL);
    if (qos_type == GST_QOS_TYPE_UNDERFLOW) {
      // TODO mafanasyev: use some other event as liveness indicator
      main_app_sl_add_stat(this->main_app_sl, "xv-qos-underflow", 1);
      return GST_PAD_PROBE_OK;
    }
  } else if ((strcmp(GST_EVENT_TYPE_NAME(event), "caps") == 0) ||
             (strcmp(GST_EVENT_TYPE_NAME(event), "segment") == 0) ||
             (strcmp(GST_EVENT_TYPE_NAME(event), "latency") == 0) ||
             (strcmp(GST_EVENT_TYPE_NAME(event), "stream-start") == 0)) {
    return GST_PAD_PROBE_OK;
  } 
    

  main_app_sl_add_stat(this->main_app_sl, "xv-event", 1);

  if (1) {
    const GstStructure* mstruct = gst_event_get_structure(event);
    char* struct_info =
        mstruct ? gst_structure_to_string(mstruct) : g_strdup("no-struct");
    g_message("imagesink unknown event (type 0x%X, name '%s'): %s",
              info->type,
              GST_EVENT_TYPE_NAME(event),
              struct_info);
    g_free(struct_info);
  }

  return GST_PAD_PROBE_OK;
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
  
  GstElement* imagesink = gst_bin_get_by_name_recurse_up(GST_BIN(this->pipeline), "info_overlay");
  if (!imagesink) {
    assert(FALSE);
  } else {
    GstPad* input_pad = gst_element_get_static_pad(imagesink, "src");
    assert(input_pad != NULL);
    gst_pad_add_probe(input_pad, 
                      GST_PAD_PROBE_TYPE_EVENT_BOTH,
                      imagesink_event,
                      this,
                      NULL);
    gst_object_unref(input_pad);
    gst_object_unref(imagesink);
  }

  // start the pipeline
  gst_element_set_state(this->pipeline, GST_STATE_PLAYING);
}
