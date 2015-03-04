#include <stdlib.h>
#include <stdio.h>

#include <sys/time.h>

#include "i2c.h"
#include "lsm_accel.h"

int main(int argc, char** argv) {

  if (i2c_init()) { exit(1); }

  if (accel_init(ACC_LSM303DLHC) < 0) {
    exit(1);
  }

  if (accel_init(ACC_LSM303D_H) < 0) {
    exit(1);
  }


  FILE* log_C0 = fopen("/tmp/acc-log-0.txt", "wb");
  FILE* log_C1 = fopen("/tmp/acc-log-1.txt", "wb");
  if (!log_C0 || !log_C1) {
    perror("Open logfile");
    exit(1);
  }

  struct timeval t0;
  gettimeofday(&t0, NULL);

  int samples[2] = { 0, 0 };
  int skipped[2] = { 0, 0 };
  while (samples[0] < 10000) {
    int channel;
    for (channel=0; channel<2; channel++) {
      accel_data_t data;

      int rv = accel_poll(
          (channel == 0) ? ACC_LSM303DLHC : ACC_LSM303D_H,
          &data);
      if (rv < 0) {
        exit(1); // failed
      }
      if (rv == 0) {
        // No data yet
        skipped[channel]++;
        continue;
      }
      if (data.errors) {
        printf("Data overrrun (0x%02X) at ch %d step %d\n",
               data.errors, channel, samples[channel]);
      }

      struct timeval ts, dt;
      gettimeofday(&ts, NULL);
      dt.tv_sec = ts.tv_sec - t0.tv_sec;
      dt.tv_usec = ts.tv_usec - t0.tv_usec;
      while (dt.tv_usec < 0) {
        dt.tv_usec += 1000000;
        dt.tv_sec -= 1;
      }

      fprintf(channel ? log_C1 : log_C0,
              "%ld.%06ld,%ld.%06ld,%d,%d,%d,%d\n",
              dt.tv_sec, dt.tv_usec,
              ts.tv_sec, ts.tv_usec,
              samples[channel], data.x, data.y, data.z);
      samples[channel]++;
    }
  }

  int i;
  for (i=0; i<2; i++) {
    printf("Channel %d: %d skipped, %d samples\n", i, skipped[i], samples[i]);
  }
  fclose(log_C0);
  fclose(log_C1);
  i2c_close();

  return 0;
}
