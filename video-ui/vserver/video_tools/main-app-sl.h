#ifndef _MAIN_APP_SL_H
#define _MAIN_APP_SL_H

#include <gst/gst.h>

/*
  This file defines common main app functionality in "slave" mode:
  - App receives text commands on stdin; if stdin is closed, this means master
    has exited and this app needs to exit, too.
  - STDOUT is machine-parsed, so all outputs are tigtly controlled.
*/

typedef struct MainAppSL {
  // Statistics entries to print.
  GHashTable* stats;

  // Runtime information fields.
  GMainLoop* loop;
  GIOChannel* stdin;

  // Option context and main group. Only exists between make() and start().
  GOptionContext* option_ctx;
  GOptionGroup* main_group;

  // Exit code to return;
  int exit_code;

  // How often to print statistics (ms)
  int opt_stat_interval;
} MainAppSL;

MainAppSL* main_app_sl_make();

// Parse options, set up signal handlers and async watchers
void main_app_sl_start(MainAppSL*, int argc, char *argv[]);

// Run the loop until interruped
void main_app_sl_loop(MainAppSL*);

// Increment given value in stats by that much
// First pointer may be NULL, in which case it is no-op.
void main_app_sl_add_stat(MainAppSL*, const char* name, int inc);

// Stop the main loop, then exit the program with specified exitcode
// may be ran before the loop is started
void main_app_sl_exit(MainAppSL*);

#endif
