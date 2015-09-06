#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/time.h>
#include <string.h>

int g_i2c_file = -1;


/*
  Bus:
i2c-4   i2c             i2c-gpio4                               I2C adapter
  Devices:
  0x19 - LSM303DLHC accelerometer interface (MiniIMU9-v2)
  0x1d - LSM303D accelerometer interface (MiniIMU9-v3, SA0=1)
  0x1e - LSM303DLHC compass interface (MiniIMU9-v2)
  0x20 - I2C GPIO on odroid shield?
  0x6b - L3GD20 + L3GD20H gyroscopes (Both IMUs -- collision)

(MiniIMU9-v3, SA0=1) is under odroid shield
(MiniIMU9-v2) is in the turret
(MiniIMU9-v3, SA0=0) may be installed in the turret instead if we need a gyro

Future (not implemeted yet):
  0x1e - LSM303D accelerometer interface (MiniIMU9-v3, SA0=0)
  0x6a - L3GD20H gyroscope (MiniIMU9-v3, SA0=0)
  0x6b - L3GD20H gyroscope (MiniIMU9-v3, SA0=1)
 */

/*
# Configure the L3GD20 gyroscope.
#  20 CTRL_REG1 - 0b00111111 ODR=95Hz Filter=25Hz all enabled
#  21 CTRL_REG2 - 0b00000000 no high pass filter
#  22 CTRL_REG3 - 0b00000000 misc options off
#  23 CTRL_REG4 - 0b10xx0000 BDU=1 FS=x
 */
uint8_t init_L3GD20[] = { 0xa0, 0x3f, 0x00, 0x00, 0x80 };

/*
   configure the LSM303DLHC accelerometer.
   (bit 7 of address is set to enable auto-increment)
#  20 CTRL_REG1_A 0x57 = 0b01010111 ODR=100Hz low_power=off all enabled
                  0x77 =            ODR=400Hz
#  21 CTRL_REG2_A 0b00000000
#  22 CTRL_REG3_A 0b00000000
#  23 CTRL_REG4_A 0b11xx1000 BDU=1 (block) BLE=1 (big) FS=xx
#                            HR=1 (high res)
FS values: 01 (0x10) = 4G
 */
uint8_t init_LSM303DLHC[] = { 0xA0, 0x57, 0x00, 0x00, 0xC8 | 0x10 };
int addr_LSM303DLHC = 0x19;

/*
   configure the LSM303D accelerometer.
   (bit 7 of address is set to enable auto-increment)
   20 CTRL1 0x67 = 0b01101111 ODR=100Hz all enabled
   21 CTRL2 0xC8 = 0b11001000 ABW=50Hz, AFS=4G

OR:
   20 CTRL1 0xA7 = 0b10101111 ODR=1600Hz all enabled
   21 CTRL2 0x08 = 0b00001000 ABW=773Hz, AFS=4G

   22 CTRL3 0x00 = 0b00000000 No interrupts
   23 CTRL4 0x00 = 0b00000000 No interrupts
   24 CTRL5 0x00 = 0b00000000 No thermometer or compass
   25 CTRL6 0x00 = 0b00000000 No compass
   26 CTRL7 0x00 = 0b00000000 No high-pass filtering

 */
uint8_t init_LSM303D[] = { 0xA0, 0x67, 0xC8, 0, 0, 0, 0, 0, 0 };
int addr_LSM303D_H = 0x1d;

int main(int argc, char** argv) {
  g_i2c_file = open("/dev/i2c-4", O_RDWR);
  if (g_i2c_file < 0) {
    perror("i2c open");
    exit(1);
  }

  int rv;

  // Use simple interface to init.
  if (ioctl(g_i2c_file, I2C_SLAVE, addr_LSM303DLHC) < 0) {
    perror("i2c set addr (1)"); exit(1);
  }

  rv = write(g_i2c_file, init_LSM303DLHC, sizeof(init_LSM303DLHC));
  if (rv != sizeof(init_LSM303DLHC)) {
    perror("write acc init (1)"); exit(1);
  }

  if (ioctl(g_i2c_file, I2C_SLAVE, addr_LSM303D_H) < 0) {
    perror("i2c set addr (2)"); exit(1);
  }

  rv = write(g_i2c_file, init_LSM303D, sizeof(init_LSM303D));
  if (rv != sizeof(init_LSM303D)) {
    perror("write acc init (2)"); exit(1);
  }

  FILE* log_C0 = fopen("/tmp/acc-log-0.txt", "wb");
  FILE* log_C1 = fopen("/tmp/acc-log-1.txt", "wb");
  if (!log_C0 || !log_C1) {
    perror("Open logfile");
    exit(1);
  }

  // Luckily for us, both accelerometers use same registers to report
  // data.
  uint8_t txdata[1] = { 0xA7 };
  uint8_t status = 0;
  uint8_t acc_out[6] = { 0 };
  struct i2c_msg p_msgs[3];
  p_msgs[0].flags = 0;
  p_msgs[0].buf = txdata;
  p_msgs[0].len = sizeof(txdata);

  p_msgs[1].flags = I2C_M_RD; // | I2C_M_NOSTART;
  p_msgs[1].buf = &status;
  p_msgs[1].len = 1;

  p_msgs[2].flags = I2C_M_RD;
  p_msgs[2].buf = acc_out;
  p_msgs[2].len = sizeof(acc_out);

  // poll accelerometer for readiness (read reg 0x27)
  struct i2c_rdwr_ioctl_data p_arg;
  p_arg.msgs = p_msgs;
  p_arg.nmsgs = 2;

  // read data itself (address is already set by poll)
  struct i2c_rdwr_ioctl_data r_arg;
  r_arg.msgs = &(p_msgs[2]);
  r_arg.nmsgs = 1;

  struct timeval t0;
  gettimeofday(&t0, NULL);

  int samples[2] = { 0, 0 };
  int skipped[2] = { 0, 0 };
  while (samples[0] < 10000) {
    int channel;
    for (channel=0; channel<2; channel++) {
      int addr = (channel == 0) ? addr_LSM303DLHC : addr_LSM303D_H;
      p_msgs[0].addr = addr;
      p_msgs[1].addr = addr;
      p_msgs[2].addr = addr;

      status = 0xBB;
      if (ioctl(g_i2c_file, I2C_RDWR, &p_arg) < 0) {
        perror("read acc status");
        exit(1);
      }

      if ((status & 0x08) == 0) {
        // No data yet
        skipped[channel]++;
        continue;
      }
      if (status != 0x0F) {
        printf("Strange status 0x%02X at ch %d step %d\n",
               status, channel, samples[channel]);
      }

      memset(acc_out, 0, sizeof(acc_out));
      if (ioctl(g_i2c_file, I2C_RDWR, &r_arg) < 0) {
        perror("read acc data");
        exit(1);
      }
      int16_t out_x = acc_out[0] | (acc_out[1] << 8);
      int16_t out_y = acc_out[2] | (acc_out[3] << 8);
      int16_t out_z = acc_out[4] | (acc_out[5] << 8);

      struct timeval ts;
      gettimeofday(&ts, NULL);
      ts.tv_usec -= t0.tv_usec;
      ts.tv_sec -= t0.tv_sec;
      while (ts.tv_usec < 0) {
        ts.tv_usec += 1000000;
        ts.tv_sec -= 1;
      }

      fprintf(channel ? log_C1 : log_C0,
              "%ld.%06ld,%d,%d,%d,%d\n",
              ts.tv_sec, ts.tv_usec,
              samples[channel], out_x, out_y, out_z);
      samples[channel]++;
    }
  }

  int i;
  for (i=0; i<2; i++) {
    printf("Channel %d: %d skipped, %d samples\n", i, skipped[i], samples[i]);
  }
  fclose(log_C0);
  fclose(log_C1);
  close(g_i2c_file);

  return 0;
}
