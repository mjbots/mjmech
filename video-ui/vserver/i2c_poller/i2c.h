#ifndef _I2C_H_
#define _I2C_H_

#include <stdint.h>

// on error, prints message and returns non-zero
int i2c_init(void);
void i2c_close();

// on error, returns negative number
int i2c_rdwr(int addr,
             uint8_t* tx_data, int tx_size,
             uint8_t* rx_data, int rx_size);
#endif
