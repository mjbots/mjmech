// Copyright 2012-2014 Josh Pieper, Mikhail Afanasyev.  All rights reserved.

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h>

#include "i2c.h"
#include "util.h"

#define I2C_SCL_PORT PORTB
#define I2C_SCL_PIN PINB
#define I2C_SCL_DDR DDRB
#define I2C_SCL_BIT 4

#define I2C_SDA_PORT PORTB
#define I2C_SDA_PIN PINB
#define I2C_SDA_DDR DDRB
#define I2C_SDA_BIT 3

#define I2C_STATE_IDLE 0
#define I2C_STATE_ACTIVE 1

static uint8_t g_i2c_state;

static void i2c_delay() {
  _delay_us(0);
}

static uint8_t i2c_sda_high() {
  I2C_SDA_DDR &= ~(1 << I2C_SDA_BIT);
  I2C_SDA_PORT |= (1 << I2C_SDA_BIT);
  return ((I2C_SDA_PIN & (1 << I2C_SDA_BIT)) != 0x00);
}

static void i2c_sda_low() {
  I2C_SDA_PORT &= ~(1 << I2C_SDA_BIT);
  I2C_SDA_DDR |= (1 << I2C_SDA_BIT);
}

static uint8_t i2c_scl_high() {
  I2C_SCL_DDR &= ~(1 << I2C_SCL_BIT);
  I2C_SCL_PORT |= (1 << I2C_SCL_BIT);
  return ((I2C_SCL_PIN & (1 << I2C_SCL_BIT)) != 0x00);
}

static void i2c_scl_low() {
  I2C_SCL_PORT &= ~(1 << I2C_SCL_BIT);
  I2C_SCL_DDR |= (1 << I2C_SCL_BIT);
}

void i2c_init(void) {
  /* Start with both pins as inputs with pullups enabled. */
  i2c_sda_high();
  i2c_scl_high();
  g_i2c_state = I2C_STATE_IDLE;
}

uint8_t i2c_start() {
  if (g_i2c_state == I2C_STATE_ACTIVE) {
    /* Handle the repeated start case. */
    i2c_sda_high();
    i2c_delay();
    i2c_scl_high();
    i2c_delay();
  } else {
    /* If this is our first go, then both lines should be high,
     * otherwise another master is using the bus. */
    if (!i2c_sda_high() || !i2c_scl_high()) {
      return I2C_ERR_BUSY;
    }
  }
  i2c_sda_low();
  i2c_delay();
  i2c_scl_low();
  i2c_delay();

  g_i2c_state = I2C_STATE_ACTIVE;
  return 0;
}

static uint8_t i2c_stop() {
  i2c_sda_low();
  i2c_delay();
  i2c_scl_high();
  i2c_delay();
  i2c_sda_high();
  i2c_delay();

  if (g_i2c_state != I2C_STATE_ACTIVE) {
    return I2C_ERR_BADCMD;
  }
  
  if (!(I2C_SCL_PIN & (1 << I2C_SCL_BIT)) ||
      !(I2C_SDA_PIN & (1 << I2C_SDA_BIT))) {
    return I2C_ERR_BUSY;
  }

  g_i2c_state = I2C_STATE_IDLE;
  
  return 0;
}

static uint8_t i2c_write_bit(uint8_t bit) {
  if (bit) {
    i2c_sda_high();
  } else {
    i2c_sda_low();
  }
  i2c_delay();

  i2c_scl_high();

  if (bit && !i2c_sda_high()) {
    return I2C_ERR_WRITE_COLL;
  }

  i2c_delay();
  i2c_scl_low();
  return 0;
}

/** Returns 0 or 1 */
static uint8_t i2c_read_bit() {
  uint8_t bit;

  i2c_sda_high();
  i2c_delay();
  i2c_scl_high();
  bit = i2c_sda_high();
  i2c_delay();
  i2c_scl_low();
  return bit;
}

/** Return 0 if success, 0xff if success and NACK. */
static uint8_t i2c_write_byte(uint8_t data) {
  if (g_i2c_state != I2C_STATE_ACTIVE) {
    return I2C_ERR_BADCMD;
  }
  
  uint8_t err = 0;
  for (uint8_t bit = 0; bit < 8; bit++) {
    err = i2c_write_bit(data & 0x80);
    if (err != 0) { return err; }
    data <<= 1;
  }

  err = i2c_read_bit();
  if (err) {
    return 0xff;
  }
  return err;
}

/** Return 0 if success. */
static uint8_t i2c_read_byte(uint8_t nack, uint8_t* data) {
  if (g_i2c_state != I2C_STATE_ACTIVE) {
    return I2C_ERR_BADCMD;
  }
  
  uint8_t result = 0;
  uint8_t err;
  for (uint8_t bit = 0; bit < 8; bit++) {
    err = i2c_read_bit();
    result <<= 1;
    if (err) {
      result |= 0x01;
    }
  }
  if ((err = i2c_write_bit(nack))) {
    return err;
  }

  *data = result;
  return 0;
}

#define I2C_CMD_START 1
#define I2C_CMD_STOP 2

static uint8_t is_hex(char val) {
  return (val >= '0' && val <= '9') ||
      (val >= 'a' && val <= 'f') ||
      (val >= 'A' && val <= 'F') ||
      (val == ' ' || val == ':');
}

static uint8_t run_command(char* arg, int* addr,
                           uint8_t* resp_buffer, uint8_t resp_size,
                           uint8_t* resp_len) {
  uint8_t err = 0;
  uint8_t val = 0;
  uint8_t mask = 0;
  uint8_t in_transaction = 0;
  uint8_t last_read = 0;
  uint8_t last_nack = 0;

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
          if ((err = i2c_start())) {
            return err;
          }

          if ((err = i2c_write_byte(((*addr) << 1) | 1))) {
            if (err == 0xff) { return I2C_ERR_ADDR_NACK; }
            return err;
          }
        }

        in_transaction = 1;
        last_read = 1;

        // Read the bytes.
        for (uint8_t i = 0; i < val; i++) {
          /* Set ACK on all bits.  We might chain up reads, and in any
           * case, the device will know we're done when we send the
           * stop condition. */
          if (final_nack && (i + 1) == val) {
            last_nack = 1;
          } else {
            last_nack = 0;
          }
          if ((err = i2c_read_byte(last_nack,
                                   resp_buffer + (*resp_len)))) {
            return err;
          }
          (*resp_len)++;
          
          if (*resp_len >= resp_size) {
            return I2C_ERR_RXBUFF_FULL;
          }
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
          if ((err = i2c_start())) { return err; }

          // Send address in read mode.
          if ((err = i2c_write_byte(*addr << 1))) {
            if (err == 0xff) {
              return I2C_ERR_ADDR_NACK;
            }
            return err;
          }
        }
        in_transaction = 1;
        last_read = 0;
        // Start sending data.
        while (is_hex(*arg)) {
          if ((err = parse_hex2(&arg, &val))) { return err; }
          if ((err = i2c_write_byte(val))) {
            if (err == 0xff) {
              return I2C_ERR_DATA_NACK;
            }
            return err;
          }
        }
        break;
      }
      case 'p': { // Check last poll status.
        arg++;
        if ((err = parse_hex2(&arg, &mask))) { return err; }
        if ((err = parse_hex2(&arg, &val))) { return err; }
        if (*resp_len == 0) {
          // This command requires data.
          return I2C_ERR_BADCMD;
        }
        if (((resp_buffer[*resp_len - 1] & mask) !=
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
        i2c_stop();
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

static const char err_no_error[] PROGMEM = "i2c-no-error";
static const char err_timeout[] PROGMEM = "i2c-timeout";
static const char err_write_coll[] PROGMEM = "i2c-write-coll";
static const char err_rxbuff_full[] PROGMEM = "i2c-rxbuff-full";
static const char err_badhex[] PROGMEM = "i2c-badhex";
static const char err_badcmd[] PROGMEM = "i2c-badcmd";
static const char err_stuck_pin[] PROGMEM = "i2c-stuck-pin";
static const char err_badaddr[] PROGMEM = "i2c-badaddr";
static const char err_busy[] PROGMEM = "i2c-busy";
static const char err_output_full[] PROGMEM = "i2c-out-full";
static const char err_addr_nack[] PROGMEM = "i2c-nack-addr";
static const char err_data_nack[] PROGMEM = "i2c-nack-data-tx";
static const char err_unknown[] PROGMEM = "i2c-unknown";

static const char* const PROGMEM err_table[] = {
  err_no_error,
  err_timeout,
  err_write_coll,
  err_rxbuff_full,
  err_badhex,
  err_badcmd,
  err_stuck_pin,
  err_busy,
  err_badaddr,
  err_output_full,
  err_addr_nack,
  err_data_nack,
};

const char* get_errstr_P(uint8_t err) {
  if (err > I2C_ERR_DATA_NACK) {
    return err_unknown;
  }

  return (const char*) pgm_read_word(err_table + err);
}

uint8_t i2c_command(char* cmd, char* response, uint8_t response_len) {
  int addr = -1;
  uint8_t data_out[10];
  uint8_t data_len = 0;
  uint8_t err = run_command(cmd, &addr, data_out, sizeof(data_out), &data_len);

  i2c_stop();

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
    for (uint8_t i = 0; i < data_len; i++) {
      if (ptr >= end) {
        strcpy_P(response, PSTR(" ERR OVERFLOW"));
        return I2C_ERR_OUTPUT_FULL;
      }
      uint8_to_hex(&ptr, data_out[i]);
    }
    *ptr = 0;
  }

  return err;
}
