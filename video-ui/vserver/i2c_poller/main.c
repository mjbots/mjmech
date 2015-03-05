#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>

#include <sys/time.h>
#include <sys/stat.h>

#include "i2c.h"
#include "lsm_accel.h"

FILE* g_data_log = NULL;
time_t g_data_log_expires = 0;
const char g_output_path[] = "/tmp/accel-log";

// Keep that many recent logs; older logs are deleted
#define RECENT_LOGNAME_COUNT   10
char* g_recent_names[RECENT_LOGNAME_COUNT];
int g_recent_names_pos = 0;

#define MAX_CHANNELS 16
struct {
  int samples;
  int skipped;
  int errors;
} g_output_stats[MAX_CHANNELS];

void dump_output_stats() {
  int i;
  int nonzero = 0;
  fprintf(stderr, "records: ");
  for (i=0; i<MAX_CHANNELS; i++) {
    if (!(g_output_stats[i].samples ||
          g_output_stats[i].skipped ||
          g_output_stats[i].errors)) { continue; }
    if (nonzero++) {
      fprintf(stderr, "; ");
    }
    fprintf(stderr, "ch%d: %d/%d", i,
            g_output_stats[i].samples, g_output_stats[i].skipped);
    if (g_output_stats[i].errors) {
      fprintf(stderr, "+%dbad", g_output_stats[i].errors);
    }
  }
  memset(g_output_stats, 0, sizeof(g_output_stats));
  if (!nonzero) {
    fprintf(stderr, "none");
  }
  fprintf(stderr, "\n");
}

void open_data_log() {
  if (g_data_log) {
    fclose(g_data_log);
  }
  time_t now_time = time(NULL);
  struct tm now_tm;
  localtime_r(&now_time, &now_tm);
  char buff[256];
  strncpy(buff, g_output_path, sizeof(buff));
  strftime(buff + strlen(buff),
           sizeof(buff) - strlen(buff),
           "/acclog-%F-%H%M%S.txt", &now_tm);
  fprintf(stderr, "Logging to '%s'; ", buff);
  dump_output_stats(); // Will add EOL

  mkdir(g_output_path, 0777); // ignore error messages
  g_data_log = fopen(buff, "w");
  if (!g_data_log) {
    perror("open-log");
    exit(1);
  }
  g_data_log_expires = now_time + (60 - now_tm.tm_sec);

  // Record file to recent names; delete old file if needed,
  g_recent_names_pos = (g_recent_names_pos + 1) % RECENT_LOGNAME_COUNT;
  char* old = g_recent_names[g_recent_names_pos];
  g_recent_names[g_recent_names_pos] = strdup(buff);
  if (old) {
    if (unlink(old) < 0) {
      fprintf(stderr, "[W] Cannot delete old file '%s': %d\n",
              old, errno);
    }
    free(old);
  }
}

void log_accel_data(int channel, accel_data_t* data) {
  if (!g_data_log) { return; };

  struct timeval ts;
  gettimeofday(&ts, NULL);
  if (ts.tv_sec >= g_data_log_expires) {
    open_data_log();
  }

  fprintf(g_data_log,
          "%ld.%06ld,%d,%.4f,%.4f,%.4f\n",
          ts.tv_sec, ts.tv_usec,
          channel, data->x, data->y, data->z);
}

volatile int g_want_exit = 0;
void handle_signal(int signum) {
  g_want_exit = signum;
}

int main(int argc, char** argv) {

  if (i2c_init()) { exit(1); }

  if ((accel_init(ACC_LSM303DLHC) < 0) ||
      (accel_init(ACC_LSM303D_H) < 0)) {
    exit(1);
  }

  struct sigaction sa;
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = handle_signal;
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGHUP, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);

  memset(g_output_stats, 0, sizeof(g_output_stats));
  open_data_log();

  while (!g_want_exit) {
    int channel;
    for (channel=0; channel<2; channel++) {
      accel_data_t data;
      int rv = accel_poll((channel == 0) ? ACC_LSM303DLHC : ACC_LSM303D_H,
                          &data);
      if (rv < 0) {
        exit(1); // failed
      }
      if (rv == 0) {
        // No data yet
        g_output_stats[channel].skipped++;
        continue;
      }
      if (data.errors && g_output_stats[channel].errors < 10) {
        g_output_stats[channel].errors++;
        fprintf(stderr,
                "[W] Accelerometer data overrrun (0x%02X) at ch %d step %d\n",
                data.errors, channel, g_output_stats[channel].samples);
      }
      g_output_stats[channel].samples++;
      log_accel_data(channel, &data);
    }
  }

  if (g_data_log) {
    fclose(g_data_log);
  }
  i2c_close();
  dump_output_stats();
  fprintf(stderr, "[E] Exiting, reason %d; ", g_want_exit);

  return g_want_exit;
}
