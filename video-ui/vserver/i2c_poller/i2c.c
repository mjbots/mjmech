#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "i2c.h"

int g_i2c_file = -1;
int i2c_init(void) {
  g_i2c_file = open("/dev/i2c-4", O_RDWR);
  if (g_i2c_file < 0) {
    perror("i2c open");
    return 1;
  }
  return 0;
}

void i2c_close() {
  close(g_i2c_file);
}

int i2c_rdwr(int addr,
              uint8_t* tx_data, int tx_size,
              uint8_t* rx_data, int rx_size) {

  struct i2c_msg msgs[2];
  struct i2c_rdwr_ioctl_data arg;
  arg.msgs = msgs;
  arg.nmsgs = 0;

  if (tx_size) {
    msgs[0].addr = addr;
    msgs[0].flags = 0;
    msgs[0].buf = tx_data;
    msgs[0].len = tx_size;
    arg.nmsgs++;
  }

  if (rx_size) {
    msgs[arg.nmsgs].addr = addr;
    msgs[arg.nmsgs].flags = I2C_M_RD;
    msgs[arg.nmsgs].buf = rx_data;
    msgs[arg.nmsgs].len = rx_size;
    arg.nmsgs++;
  }

  // Return if nothing to do
  if (!arg.nmsgs) { return 0; };

  return ioctl(g_i2c_file, I2C_RDWR, &arg);
}
