#include <assert.h>
#include <stdlib.h>
#include <stdio.h>

#include <gst/gst.h>
#include <glib-unix.h>

#include "camera-recv.h"
#include "rtsp-server.h"


/*
To test playback:

gst-launch-1.0 rtspsrc location=rtsp://mjmech-odroid:8554/video latency=200 ! decodebin ! timeoverlay ! videoconvert ! xvimagesink
vlc --rtsp-caching=200  rtsp://127.0.0.1:8554/video
mpv --cache-secs=0.2 rtsp://127.0.0.1:8554/video

*/

typedef struct MainCtx {
  CameraReceiver* receiver;
  RtspServer* server;
  GMainLoop* loop;
  GIOChannel* stdin;
} MainCtx;

static void exit_app(MainCtx* this, char* reason) {
  g_message("Exiting: %s", reason);
  g_main_loop_quit(this->loop);
}

// No (simple) lambdas here...
static gboolean handle_sighup(void* userarg) {
  exit_app((MainCtx*)userarg, "Signal: SIGHUP");
  return FALSE; // only handle once
}
static gboolean handle_sigint(void* userarg) {
  exit_app((MainCtx*)userarg, "Signal: SIGINT");
  return FALSE; // only handle once
}
static gboolean handle_sigterm(void* userarg) {
  exit_app((MainCtx*)userarg, "Signal: SIGTERM");
  return FALSE; // only handle once
}

static gboolean stdin_readable(
    GIOChannel *gio, GIOCondition condition, void* userarg) {
  MainCtx* this = (MainCtx*)userarg;
  if (condition & G_IO_HUP) {
    exit_app(this, "HUP on stdin");
    return TRUE;
  }

  GError *err = NULL;
  gchar *msg = NULL;
  gsize len = 0;
  gsize terminator = 0;
  GIOStatus ret = g_io_channel_read_line(gio, &msg, &len, &terminator, &err);
  if (ret == G_IO_STATUS_ERROR) {
    char* errtext = g_strdup_printf("Error reading stdin: %s", err->message);
    exit_app(this, errtext);
    g_free(errtext);
    return TRUE;
  }
  if (len == 0) {
    exit_app(this, "EOF on stdin");
    return TRUE;
  }

  // Discard EOL character
  msg[terminator] = 0;
  g_message("stdin: ignoring command [%s]", msg);
  g_free(msg);
  return TRUE;
}


int main (int argc, char *argv[])
{
  MainCtx* this = g_malloc0(sizeof(MainCtx));

  this->receiver = camera_receiver_make();
  assert(this->receiver != NULL);
  // Create and register RTSP server
  this->server = rtsp_server_make();
  assert(this->server != NULL);

  {
    GOptionContext* ctx = g_option_context_new("");
    // Use one shared group -- we do not have enough options to require
    // a group per .c file.
    GOptionGroup* group = g_option_group_new(
        NULL, "Main options", "Show main options", NULL, NULL);
    camera_receiver_add_options(this->receiver, group);
    rtsp_server_add_options(this->server, group);
    g_option_context_set_main_group(ctx, group);

    //g_option_context_add_main_entries(ctx, options, NULL);
    g_option_context_add_group(ctx, gst_init_get_option_group());

    GError* err = NULL;
    if (!g_option_context_parse(ctx, &argc, &argv, &err)) {
      fprintf(stderr, "Error initializing: %s\n", GST_STR_NULL(err->message));
      exit(1);
    }
    g_option_context_free(ctx);

    if (argc > 1) {
      fprintf(stderr, "No positional arguments accepted\n");
      exit(1);
    }

    gst_init(&argc, &argv);
  }

  // Exit on critical messages from our app
  g_log_set_fatal_mask(NULL, G_LOG_LEVEL_ERROR | G_LOG_LEVEL_CRITICAL);
  // Exit on critical messages from GStreamer
  g_log_set_fatal_mask("GStreamer", G_LOG_LEVEL_ERROR | G_LOG_LEVEL_CRITICAL);

  this->loop = g_main_loop_new(NULL, FALSE);

  // Create stdin reader.
  // We are run as a subprocess, so we exit when stdin is closed.
  this->stdin = g_io_channel_unix_new(0);
  gboolean ok =
      g_io_add_watch(this->stdin, G_IO_IN | G_IO_HUP, stdin_readable, this);
  assert(ok);

  // Create signal handlers.
  g_unix_signal_add(SIGHUP, handle_sighup, this);
  g_unix_signal_add(SIGINT, handle_sigint, this);
  g_unix_signal_add(SIGTERM, handle_sigterm, this);

  this->receiver->rtsp_server = this->server;

  camera_receiver_start(this->receiver);
  rtsp_server_start(this->server);

  g_main_loop_run(this->loop);
  return 0;
}
