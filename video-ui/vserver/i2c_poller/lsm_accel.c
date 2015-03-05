#include "lsm_accel.h"

#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "i2c.h"


/*
   configure the LSM303DLHC accelerometer.
   (bit 7 of address is set to enable auto-increment)
  20 CTRL_REG1_A 0x57 = 0b01010111 ODR=100Hz low_power=off all enabled
                 0x77 =            ODR=400Hz
  21 CTRL_REG2_A 0b00000000
  22 CTRL_REG3_A 0b00000000
  23 CTRL_REG4_A 0b11xx1000 BDU=1 (block) BLE=1 (big) FS=xx
                            HR=1 (high res)
FS values: 01 (0x10) = 4G
addr_LSM303DLHC = 0x19;
 */
static uint8_t init_LSM303DLHC[] = { 0xA0, 0x57, 0x00, 0x00, 0xC8 | 0x10 };

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

addr_LSM303D_H = 0x1d;
addr_LSM303D_L = 0x1e;
 */
static uint8_t init_LSM303D[] = { 0xA0, 0x67, 0xC8, 0, 0, 0, 0, 0, 0 };

// Full scale is 4G in both cases
#define FULL_SCALE  (4.0 / 32767)

int accel_init(int atype) {
  int rv = -1;
  switch (atype) {
    case ACC_LSM303DLHC: {
      rv = i2c_rdwr(atype,
                    init_LSM303DLHC, sizeof(init_LSM303DLHC),
                    NULL, 0);
      break;
    }
    case ACC_LSM303D_H:
    case ACC_LSM303D_L: {
      rv = i2c_rdwr(atype,
                    init_LSM303D, sizeof(init_LSM303D),
                    NULL, 0);
      break;
    }
    default: {
      rv = -1;
      errno = EINVAL;
    }
  }
  if (rv < 0) {
    fprintf(stderr, "Error initializing 0x%02X: %d %d %s\n",
            atype, rv, errno, strerror(errno));
  }
  return rv;
}

// Poll an accelerometer
// Returns negative for error, 0 for no data, 1 if data retrieved
int accel_poll(int atype, accel_data_t* data) {
  // Luckily for us, both accelerometers use same registers to report
  // data.
  uint8_t txdata[1] = { 0xA7 };
  uint8_t status = 0;
  if (i2c_rdwr(atype, txdata, sizeof(txdata), &status, 1) < 0) {
    perror("read acc status");
    return -1;
  }
  if ((status & 0x08) == 0) {
    return 0;
  }
  data->errors = status ^ 0xF;

  uint8_t acc_out[6] = { 0 };
  if (i2c_rdwr(atype, NULL, 0, acc_out, sizeof(acc_out)) < 0) {
    perror("read acc data");
    return -1;
  }
  data->x = ((int16_t)(acc_out[0] | (acc_out[1] << 8))) * FULL_SCALE;
  data->y = ((int16_t)(acc_out[2] | (acc_out[3] << 8))) * FULL_SCALE;
  data->z = ((int16_t)(acc_out[4] | (acc_out[5] << 8))) * FULL_SCALE;
  return 1;
}
