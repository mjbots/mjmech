// Copyright 2012-2013 Josh Pieper, Mikhail Afanasyev.  All rights reserved.

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/twi.h>
#include <string.h>

#include "i2c.h"
#include "util.h"

static uint8_t g_i2c_resp_len = 0;
static uint8_t g_i2c_resp_buffer[64];

void i2c_init(void) {
  // SCL freq = F_CPU / ((16 + 2 * TWBR) * 4 ^ (TWPS))
  TWSR = 0; // Set TWPS to 0
  TWBR = 12; // 152 = 50kHz, 72 = 100kHz, 32=200kHz, 12=400kHz
}

/** Wait until either a timeout, or the twi operation is done.  On
 * error, returns the appropriate error code, otherwise returns 0. */
static uint8_t i2c_transact(uint8_t cmd,
                            uint8_t exp_status, uint8_t exp_status2) {
  TWCR = (1<<TWINT) | (1<<TWEN) | cmd;

  while (!(TWCR & (1<<TWINT))) {
    if (TIFR2 & (1 << TOV2)) {
      return I2C_ERR_TIMEOUT;
    }
  }
  if (TWCR & (1<<TWWC)) {
    return I2C_ERR_WRITE_COLL;
  }

  if (exp_status != (TWSR & 0xF8) && exp_status2 != (TWSR & 0xF8)) {
    return TWSR & 0xF8;
  }
  return 0;
}

static uint8_t is_hex(char val) {
  return (val >= '0' && val <= '9') ||
      (val >= 'a' && val <= 'f') ||
      (val >= 'A' && val <= 'F') ||
      (val == ' ' || val == ':');
}

static uint8_t run_command(char* arg, int* addr) {
  uint8_t err = 0;
  uint8_t val = 0;
  uint8_t mask = 0;
  uint8_t in_transaction = 0;
  uint8_t last_read = 0;
  uint8_t last_nack = 0;

  // check for stuck pins
  if ((PIND & 3) != 3) {
    // try to recover first.
    TWCR = 0;

    // clock many clocks.
    for (uint8_t i = 0; i < 35; i++) {
      PORTD &=~ (1<<0); DDRD |= (1<<0);
      _delay_loop_2(1000);
      DDRD &=~ (1<<0);  PORTD |= (1<<0);
      _delay_loop_2(1000);
    }
    // still stuck?
    if ((PIND & 3) != 3) {
      return I2C_ERR_STUCK_PIN;
    }
  }

  while (*arg) {
    switch (*arg) {
      case 's': {
        arg++;
        if ((err = parse_hex2(&arg, &val))) { return err; }
        *addr = val;
        break;
      }
      case 'n':   // read terminating in a NACK, fall through
      case 'r': { // read, should be followed by length
        uint8_t final_nack = (*arg == 'r');
        arg++;

        // Read in the length.
        if ((err = parse_hex2(&arg, &val))) { return err; }
        // Verify address.
        if (*addr < 0 || *addr > 0x7f) { return I2C_ERR_BADADDR; }
        if (in_transaction == 0 ||
            last_read != 1) {
          // Start transaction.
          if ((err = i2c_transact((1 << TWSTA), TW_START, TW_REP_START))) {
            return err;
          }
          // Send address, in read mode.
          TWDR = ((*addr) << 1) | 1;
          if ((err = i2c_transact(0, TW_MR_SLA_ACK, 1))) { return err; }
        }

        in_transaction = 1;
        last_read = 1;

        // Read the bytes.
        for (uint8_t i = 0; i < val; i++) {
          /* Set ACK on all bits.  We might chain up reads, and in any
           * case, the device will know we're done when we send the
           * stop condition. */
          uint8_t cmd = 0;
          if (final_nack && (i + 1) == val) {
            cmd = 0;
            last_nack = 1;
          } else {
            cmd = 1 << TWEA;
            last_nack = 0;
          }
          if ((err = i2c_transact(cmd, TW_MR_DATA_ACK, TW_MR_DATA_NACK))) {
            return err;
          }
          if (g_i2c_resp_len >= (sizeof(g_i2c_resp_buffer))) {
            return I2C_ERR_RXBUFF_FULL;
          }

          g_i2c_resp_buffer[g_i2c_resp_len++] = TWDR;
        }
        if (final_nack) {
          in_transaction = 0;
        }
        break;
      }
      case 'w': { // write, followed by pairs of hex digits
        arg++;
        if (*addr < 0 || *addr > 0x7f) {
          return I2C_ERR_BADADDR;
        }
        if (in_transaction == 0 ||
            last_read != 0) {
          // Start the transaction.
          if ((err = i2c_transact((1 << TWSTA), TW_START, TW_REP_START))) {
            return err;
          }
          // Send address in read mode.
          TWDR = (*addr << 1);
          if ((err = i2c_transact(0, TW_MT_SLA_ACK, 1))) { return err; }
        }
        in_transaction = 1;
        last_read = 0;
        // Start sending data.
        while (is_hex(*arg)) {
          if ((err = parse_hex2(&arg, &val))) { return err; }
          TWDR = val;
          if ((err = i2c_transact(0, TW_MT_DATA_ACK, 1))) { return err; }
        }
        break;
      }
      case 'p': { // Check last poll status.
        arg++;
        if ((err = parse_hex2(&arg, &mask))) { return err; }
        if ((err = parse_hex2(&arg, &val))) { return err; }
        if (g_i2c_resp_len == 0) {
          // This command requires data.
          return I2C_ERR_BADCMD;
        }
        if (((g_i2c_resp_buffer[g_i2c_resp_len - 1] & mask) !=
             (val & mask))) {
          if (!last_nack) { return 0xfe; }
          return 0xff;
        }
        break;
      }
      case ' ': // fall-through
      case ':': {
        // Ignore these.
        arg++;
        break;
      }
      case ';': {
        // Send a STOP condition.
        TWCR = (1 << TWINT) | (1 << TWEN) || (1 << TWSTO);
        _delay_loop_2(1000);
        arg++;
        in_transaction = 0;
        break;
      }
      default: {
        return I2C_ERR_BADCMD;
      }
    }
  }
  return 0;
}

static char* get_errstr_P(uint8_t err) {
  switch (err) {
    // custom
    case I2C_ERR_TIMEOUT: { return PSTR("i2c-timeout");  }
    case I2C_ERR_WRITE_COLL: { return PSTR("i2c-write-coll"); }
    case I2C_ERR_RXBUFF_FULL: { return PSTR("i2c-rxbuff-full"); }
    case I2C_ERR_BADHEX: { return PSTR("i2c-badhex"); }
    case I2C_ERR_BADCMD: { return PSTR("i2c-badcmd"); }
    case I2C_ERR_BADADDR: { return PSTR("i2c-badaddr"); }
    case I2C_ERR_STUCK_PIN: { return PSTR("i2c-stuck-pin"); }
    case I2C_ERR_BUSY: { return PSTR("i2c-busy"); }

    // stock
    case TW_MT_SLA_NACK: { return PSTR("i2c-nack-addr-tx"); }
    case TW_MT_DATA_NACK: { return PSTR("i2c-nack-data-tx"); }
    case TW_MR_SLA_NACK: { return PSTR("i2c-nack-addr-rx"); }

      // others
    case 0: { return PSTR("i2c-no-error"); }
    default: { return PSTR("i2c-unknown"); }
  }
}

uint8_t i2c_command(char* cmd, char* response, uint8_t response_len) {
  g_i2c_resp_len = 0;

  TCCR2B = 0x00; // ensure timer2 is stopped
  TCNT2 = 0xe0;
  TCCR2B = 0x06; // 256 prescale * 32 (0xff-0xe0) = about 0.5ms timeout
  TIFR2 |= (1 << TOV2);

  int addr = -1;
  uint8_t err = run_command(cmd, &addr);

  if (err == 0xfe) {
    /* We ended early and haven't done a final NACK read yet.  Do one
       now, as the AVR TWI appears to require that the final read be
       NACK'd or it cannot emit a stop condition. */
    uint8_t discard = 0;
    i2c_transact(0, TW_MR_DATA_ACK, TW_MR_DATA_NACK);

    discard = TWDR;

    err = 0xff;
  }

  // Always send a stop, regardless of error state.
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);

  // Stop timer2.
  TCCR2B = 0x00;

  if (err == 0xff) {
    strcat_P(response, PSTR(" poll early"));
  } else if (err != 0) {
    // Formulate error string.
    strcat_P(response, PSTR(" ERR "));
    strcat_P(response, get_errstr_P(err));
  } else {
    // Formulate response string.
    strcat_P(response, PSTR(" OK "));
    char* ptr = response + strlen(response);
    char* end = response + response_len;
    for (uint8_t i = 0; i < g_i2c_resp_len; i++) {
      if (ptr >= end) {
        strcpy_P(response, PSTR(" ERR OVERFLOW"));
        return I2C_ERR_OUTPUT_FULL;
      }
      uint8_to_hex(&ptr, g_i2c_resp_buffer[i]);
    }
    *ptr = 0;
  }

  return err;
}
