// Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

#pragma once

/***********************************************************
 * Serial Module
 *
 * Resources:
 *  + USART1
 *  + N bytes of RAM
 */

/** Initialize the serial subsystem. */
void serial_init(void);

#define SERIAL_PARITY_NONE 0
#define SERIAL_PARITY_EVEN 2
#define SERIAL_PARITY_ODD 3

#define SERIAL_ERR_NONE 0
#define SERIAL_ERR_INVALID_BAUD 1
#define SERIAL_ERR_INVALID_PARITY 2
#define SERIAL_ERR_INVALID_CHAR_SIZE 3
#define SERIAL_ERR_INVALID_STOP_BITS 4
#define SERIAL_ERR_OVERFLOW 5


#define SERIAL_ERR_RX_OVERFLOW (1 << 0)

/** Configure the serial subsystem.  Return 0 on success, or non-zero
 * on failure. */
uint8_t serial_configure(uint32_t baud_rate,
                         uint8_t char_size,
                         uint8_t parity,
                         uint8_t stop_bits);

/** Return non-zero if data is available to read. */
uint8_t serial_available(void);

/** Return the next byte of data. -1 if timeout/error */
int16_t serial_getchar(void);

/** Write a single byte. Return non-zero on error.*/
uint8_t serial_putchar(uint8_t);

/** Write a buffer. */
uint8_t serial_write(const uint8_t*, uint16_t size);

/** The following retrieve the current configured state. */
uint32_t serial_get_baud(void);
uint8_t serial_get_char_size(void);
uint8_t serial_get_parity(void);
uint8_t serial_get_stop_bits(void);
