#include <assert.h>
#include <stdlib.h>

#include "main-app-sl.h"
#include "video-client.h"



int main (int argc, char *argv[])
{
  MainAppSL* mainapp = main_app_sl_make();
  VideoClient* cli = video_client_make();

  video_client_add_options(cli, mainapp->main_group);
  cli->main_app_sl = mainapp;

  main_app_sl_start(mainapp, argc, argv);
  video_client_start(cli);

  main_app_sl_loop(mainapp);

  return mainapp->exit_code;
}
