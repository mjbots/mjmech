// Copyright 2012 Josh Pieper.  All rights reserved.

#pragma once

/***********************************************************
 * I2C Module
 *
 * Resources:
 *  + Timer2
 *  + 65 bytes of RAM
 */

/** Initialize the I2C subsystem. */
void i2c_init(void);

/** Error codes from the last I2C operation.  0 is success, even
 * values >=8 are from avr/include/util/twi.h. */
#define I2C_ERR_TIMEOUT     0x01
#define I2C_ERR_WRITE_COLL  0x03
#define I2C_ERR_RXBUFF_FULL 0x05
#define I2C_ERR_BADHEX      0x07
#define I2C_ERR_BADCMD      0x09
#define I2C_ERR_STUCK_PIN   0x0A
#define I2C_ERR_BUSY        0x0C
#define I2C_ERR_BADADDR     0x0E
#define I2C_ERR_OUTPUT_FULL 0x11

/** Execute a single I2C command.
 *
 * @return 0 on success, or 0xff if a 'p' command resulted in halting
 * the command.  Otherwise return a valid error code.
 *
 * response will be filled in with a NULL terminated string
 */
uint8_t i2c_command(char* cmd, char* response, uint8_t response_len);

