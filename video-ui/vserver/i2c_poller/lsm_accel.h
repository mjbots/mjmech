#ifndef _LSM_ACCEL_H_
#define _LSM_ACCEL_H_

#include <stdint.h>

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

enum {
      // 'atype' values for accel_ functions. The fact that they look like i2c
      // addresses is implementation detail, and should not be relied on.
      ACC_LSM303DLHC = 0x19,
      ACC_LSM303D_H = 0x1d,
      ACC_LSM303D_L = 0x1e,
    };

// Return negative on error. atype is one of ACC_ constants
int accel_init(int atype);

typedef struct accel_data {
  // Non-zero value means buffer overrun
  int16_t errors;
  float x;
  float y;
  float z;
} accel_data_t;

// Poll an accelerometer
// Returns negative for error, 0 for no data, 1 if data retrieved
int accel_poll(int atype, accel_data_t* data);

#endif
