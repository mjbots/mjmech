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

#include <gst/gst.h>

#include "camera-recv.h"
#include "rtsp-server.h"


int main (int argc, char *argv[])
{
  gst_init(&argc, &argv);

  // Exit on critical messages
  g_log_set_fatal_mask("", G_LOG_LEVEL_ERROR | G_LOG_LEVEL_CRITICAL);

  GMainLoop* loop = g_main_loop_new(NULL, FALSE);

  CameraReceiver* receiver = camera_receiver_make();
  assert(receiver != NULL);

  // Create and register RTSP server
  RtspServer* server = rtsp_server_make();
  assert(server != NULL);

  receiver->rtsp_server = server;

  camera_receiver_start(receiver);
  rtsp_server_start(server);

  g_main_loop_run(loop);
  return 0;
}
