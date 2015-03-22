/* GStreamer
 * Copyright (C) 2008 Wim Taymans <wim.taymans at gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

#include "assert.h"

#include "rtsp-server.h"

int main (int argc, char *argv[])
{
  gst_init(&argc, &argv);

  // Exit on critical messages
  g_log_set_fatal_mask("", G_LOG_LEVEL_ERROR | G_LOG_LEVEL_CRITICAL);

  RtspServer* server = make_rtsp_server();
  assert(server != NULL);
  GMainLoop* loop = g_main_loop_new(NULL, FALSE);
  g_main_loop_run(loop);
  return 0;
}
