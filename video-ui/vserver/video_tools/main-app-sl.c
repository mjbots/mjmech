#include "main-app-sl.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <glib-unix.h>

static void exit_app(MainAppSL* this, char* reason) {
  this->exit_code = 1;
  g_message("Exiting: %s", reason);
  g_main_loop_quit(this->loop);
}

// No (simple) lambdas here...
static gboolean handle_sighup(void* userarg) {
  exit_app((MainAppSL*)userarg, "Signal: SIGHUP");
  return FALSE; // only handle once
}
static gboolean handle_sigint(void* userarg) {
  exit_app((MainAppSL*)userarg, "Signal: SIGINT");
  return FALSE; // only handle once
}
static gboolean handle_sigterm(void* userarg) {
  exit_app((MainAppSL*)userarg, "Signal: SIGTERM");
  return FALSE; // only handle once
}

static gboolean stdin_readable(
    GIOChannel *gio, GIOCondition condition, void* userarg) {
  MainAppSL* this = (MainAppSL*)userarg;
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


// Increment given value in stats by that much
// First pointer may be NULL, in which case it is no-op.
void main_app_sl_add_stat(MainAppSL* this, const char* name, int inc) {
  if (!this) { return; }

  int* val = g_hash_table_lookup(this->stats, name);
  if (!val) {
    val = g_malloc0(sizeof(int));
    g_hash_table_insert(this->stats, g_strdup(name), val);
  }
  *val += inc;
}

static gboolean print_stats(void* user_arg) {
  MainAppSL* this = (MainAppSL*)user_arg;
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


MainAppSL* main_app_sl_make() {
  MainAppSL* this = g_malloc0(sizeof(MainAppSL));
  this->stats = g_hash_table_new_full(g_str_hash, g_str_equal, g_free, g_free);
  this->option_ctx = g_option_context_new("");
  this->main_group = g_option_group_new(
        NULL, "Main options", "Show main options", NULL, NULL);
  this->opt_stat_interval = 5000;

  // main_app_sl_add_group:
  GOptionEntry entries[] = {
    { "stat-interval", 'i', 0, G_OPTION_ARG_INT, &this->opt_stat_interval,
      "How often to print statistics, 0 to disable", "ms" },
    { NULL },
  };
  g_option_group_add_entries(this->main_group, entries);

  return this;
}

// Set up signal handlers and async watchers
void main_app_sl_start(MainAppSL* this, int argc, char *argv[]) {

  g_option_context_set_main_group(this->option_ctx, this->main_group);
  this->main_group = NULL;
  g_option_context_add_group(this->option_ctx, gst_init_get_option_group());

  GError* err = NULL;
  if (!g_option_context_parse(this->option_ctx, &argc, &argv, &err)) {
    fprintf(stderr, "Error initializing: %s\n", GST_STR_NULL(err->message));
    exit(1);
  }
  g_option_context_free(this->option_ctx);

  if (argc > 1) {
    fprintf(stderr, "No positional arguments accepted\n");
    exit(1);
  }

  gst_init(&argc, &argv);

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

  // start stats timer
  if (this->opt_stat_interval > 0) {
    g_timeout_add(this->opt_stat_interval, print_stats, this);
  }
}


// Run the loop until quit
void main_app_sl_loop(MainAppSL* this) {
  if (this->exit_code != 0) {
    g_warning("Not staring main loop -- init failed");
    return;
  }
  g_main_loop_run(this->loop);
}

void main_app_sl_exit(MainAppSL* this) {
  exit_app(this, "Application error");
}
