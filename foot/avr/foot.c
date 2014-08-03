/** Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved. */

#include <avr/eeprom.h>
#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>

#include <ctype.h>
#include <stdio.h>

#include "i2c.h"
#include "util.h"
#include "vcs_version.h"

#define UART_WRITE_PORT PORTB
#define UART_WRITE_BIT 1
#define UART_READ_PORT PINB
#define UART_READ_BIT 0

#define UART_CPU_FREQUENCY F_CPU
#define UART_BAUD_RATE 38400

#define UART_OUTPUT_INVERTED 1

#include "compact_software_uart.h"

//                 --------------
//        RXD    --|PB0      PA0|-- 
//        TXD    --|PB1      PA1|-- 
//               --|PB2      PA2|-- 
//               --|PB3      PA3|-- 
//               --|VCC      GND|--
//               --|GND     AVCC|--
//               --|PB4      PA4|-- 
//               --|PB5      PA5|-- 
//               --|PB6      PA6|-- 
//       Reset   --|PB7      PA7|-- 
//                 --------------

#define XSTR(s) STR(s)
#define STR(s) #s
#define DEV_VERSION "foot" FOOT_DBG_FLAG " v0.1 r" XSTR(VCS_REVISION)
#define EOL "\r\n"

#define STR_OK " OK"
#define STR_PARSE_ERROR " no parse"
#define STR_CODE_MISMATCH " code mismatch"
#define STR_INVALID_PORT " inv port"
#define STR_INVALID_BIT " inv bit"
#define STR_INVALID_VALUE " inv value"
#define STR_UNKNOWN_CMD " unk cmd"
#define STR_NOT_STREAMABLE " not stmable"
#define STR_ID_ZERO " stm id zero"
#define STR_ID_INVALID " stm id invalid"
#define STR_STREAM_ID_IN_USE " stm id in use"
#define STR_STREAM_TABLE_FULL " stm table full"
#define STR_STREAM_NOT_FOUND " stm not found"
#define STR_OVERFLOW " overflow"

static int16_t g_next_char = -1;
static uint16_t g_timer;
static uint16_t g_main_loop_count;

static int8_t find_message_index(char*);

static uint8_t cmd_png(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream) {
  strcpy_P(output, PSTR(STR_OK));
  return 0;
}

static uint8_t cmd_stb(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream);
static uint8_t cmd_ste(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream);
static uint8_t cmd_stl(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream);
static uint8_t cmd_stg(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream);
static uint16_t stream_list_bitmask(void);

static uint8_t cmd_ver(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream) {
  strcpy_P(output, PSTR(" " DEV_VERSION));
  return 0;
}

static uint8_t cmd_rst(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream) {
  for (;;) {
    PORTA ^= 0x01;
  }
  return 0;
}

static uint8_t cmd_tmq(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream) {
  *output++ = ' ';
  uint16_to_hex(&output, g_timer);
  *output = 0;
  return 0;
}

static uint8_t cmd_qgr(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream) {
  char* output_start = output;

  char* arg = extra_args;
  while (*arg != 0) {
    /* Check for overflow.  Assume no item can add more than 7
     * characters plus the NULL terminator. */
    if (output - output_start > (output_len - 8)) {
      *output = 0;
      return 1;
    }

    switch (*arg) {
      case ' ': { arg++; break; }
      case 't': {
        *output++ = ' ';
        *output++ = 't';
        uint16_to_hex(&output, g_timer);
        arg++;
        break;
      }
      case 'f': {
        *output++ = ' ';
        *output++ = 'f';
        uint16_to_hex(&output, g_main_loop_count);
        arg++;
        break;
      }
      case 'r': {
        *output++ = ' ';
        *output++ = 'r';
        /* TODO jpieper: Implement error codes. */
        uint16_to_hex(&output, 0);
        arg++;
        break;
      }
      case 'l': {
        *output++ = ' ';
        *output++ = 'l';
        uint16_to_hex(&output, stream_list_bitmask());
        arg++;
        break;
      }
      case 'p': {
        *output++ = ' ';
        *output++ = 'p';
        arg++;
        *output++ = *arg;
        char port_char = toupper(*arg);
        if (port_char < 'A' || port_char > 'B') {
          *output = 0;
          return 1;
        }

        uint8_t value = 0;
        switch (port_char) {
          case 'A': { value = PINA; break; }
          case 'B': { value = PINB; break; }
        }

        uint8_to_hex(&output, value);

        arg++;
        break;
      }
      default: {
        *output = 0;
        return 1;
      }
    }
  }

  *output = 0;
  return 0;
}

static uint8_t cmd_prf(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream) {
  *output++ = ' ';

  uint16_to_hex(&output, g_main_loop_count);

  *output = 0;
  return 0;
}

static uint8_t cmd_i2c(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream) {
  uint8_t err = i2c_command(extra_args, output, output_len);
  return err;
}

static uint8_t cmd_gpq(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream) {
  *output++ = ' ';
  uint8_to_hex(&output, PINA);
  *output++ = ' ';
  uint8_to_hex(&output, PINB);
  *output = 0;
  return 0;
}

static uint8_t cmd_gpc(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream) {
  char port = 0;
  int16_t bit = 0;
  int16_t value = 0;
  int parsed = 0;
  int len = strlen(extra_args);
  if (sscanf_P(extra_args, PSTR(" %c %x %x%n"),
               &port, &bit, &value, &parsed) != 3 ||
      parsed != len) {
    strcpy_P(output, PSTR(STR_PARSE_ERROR));
    return 1;
  }
  if (port != 'A' && port != 'a') {
    strcpy_P(output, PSTR(STR_INVALID_PORT));
    return 1;
  }
  if (bit < 0 || bit > 7) {
    strcpy_P(output, PSTR(STR_INVALID_BIT));
    return 1;
  }
  if (value != 0 && value != 1) {
    strcpy_P(output, PSTR(STR_INVALID_VALUE));
    return 1;
  }
  if (value) {
    PORTA |= (1 << bit);
  } else {
    PORTA &= ~(1 << bit);
  }

  strcpy_P(output, PSTR(STR_OK));
  return 0;
}

static uint8_t cmd_wdt(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream) {
  for (;;);
  return 0;
}

static uint8_t cmd_wpc(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream) {
  char* ptr = output;
  *ptr++ = ' ';
  uint16_t val = eeprom_read_word(0);
  /* The PC is stored in reverse order, and is off by a factor of 2
   * from what avr-objdump reports. */
  val = 2 * (((val & 0xff) << 8) | (val >> 8));
  uint16_to_hex(&ptr, val);
  *ptr = 0;
  return 0;
}

typedef uint8_t (*cmd_func_ptr)(char *, char*, uint8_t, uint8_t);

struct fw_command_struct {
  /* 3 letter code for this command. */
  char code[3];

  /* function to call to handle this command.
   *
   *  @p extra_args -- points to a NULL terminated string of extra data
   *  @p output -- points to a buffer to hold result
   *  @p output_len -- length of output buffer
   *  @p stream -- 1 if called streaming
   *
   *  @returns 0 on success, or other code on error.  Additional error
   *  text can still be entered in @p output.
   */
  cmd_func_ptr func;

  /* bitfield
   *  0 - streamable
   */
  uint8_t flags;
};

#define CMD_FLAGS_STREAMABLE 0

static const struct fw_command_struct PROGMEM fw_command_table[] = {
  { "PNG", cmd_png, 0 },
  { "STB", cmd_stb, 0 },
  { "STE", cmd_ste, 0 },
  { "STL", cmd_stl, (1 << CMD_FLAGS_STREAMABLE) },
  { "STG", cmd_stg, 0 },
  { "VER", cmd_ver, 0 },
  { "RST", cmd_rst, 0 },
  { "TMQ", cmd_tmq, (1 << CMD_FLAGS_STREAMABLE) },
  { "QGR", cmd_qgr, (1 << CMD_FLAGS_STREAMABLE) },
  { "PRF", cmd_prf, (1 << CMD_FLAGS_STREAMABLE) },
  { "I2C", cmd_i2c, (1 << CMD_FLAGS_STREAMABLE) },
  { "GPQ", cmd_gpq, (1 << CMD_FLAGS_STREAMABLE) },
  { "GPC", cmd_gpc, 0 },
  { "WDT", cmd_wdt, 0 },
  { "WPC", cmd_wpc, 0 },
};

#define NUM_FW_COMMAND (sizeof(fw_command_table) / sizeof(*fw_command_table))

static int8_t find_message_index(char* code) {
  int8_t index = 0;
  for (; index < NUM_FW_COMMAND; index++) {
    const struct fw_command_struct* fw_command = &fw_command_table[index];
    if (strncmp_P(code, fw_command->code, 3) == 0) {
      return index;
    }
  }
  return -1;
}

struct stream_struct {
  uint8_t stream_id;
  uint8_t cmd_index;
  char* extra_args;
  uint16_t rate;
  uint16_t time_remaining;
  uint8_t queued_runs;
} g_stream_table[4];

#define NUM_STREAM_TABLE (sizeof(g_stream_table) / sizeof(*g_stream_table))

/* We can support up to N streams that require arguments.  The
 * argument is in use if the first byte is non-NULL. */
#define NUM_STREAM_EXTRA_ARGS 8
char g_stream_args[NUM_STREAM_EXTRA_ARGS][64];
char g_null_args[] = "";

static uint8_t cmd_stb(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream) {
  uint16_t stream_id = 0;
  uint16_t rate = 0;
  char code[3];
  int parsed = 0;
  memset(code, 0, sizeof(code));
  if (sscanf_P(extra_args, PSTR(" %x %x %c%c%c%n"),
               &stream_id, &rate,
               &code[0], &code[1], &code[2], &parsed) != 5) {
    strcpy_P(output, PSTR(STR_PARSE_ERROR));
    return 1;
  }

  int8_t message_index = find_message_index(code);
  if (message_index < 0) {
    strcpy_P(output, PSTR(" stream" STR_UNKNOWN_CMD));
    return 1;
  }

  const struct fw_command_struct* fw_command = &fw_command_table[message_index];
  uint8_t flags = pgm_read_byte(&fw_command->flags);
  if (!(flags & (1 << CMD_FLAGS_STREAMABLE))) {
    strcpy_P(output, PSTR(STR_NOT_STREAMABLE));
    return 1;
  }

  if (stream_id == 0) {
    strcpy_P(output, PSTR(STR_ID_ZERO));
    return 1;
  }

  if (stream_id > 15) {
    strcpy_P(output, PSTR(STR_ID_INVALID));
    return 1;
  }

  if (rate == 0) { rate = 1; }

  uint16_t time_remaining = rate;

  uint8_t index = 0;
  for (index = 0; index < NUM_STREAM_TABLE; index++) {
    if (g_stream_table[index].stream_id == stream_id) {
      strcpy_P(output, PSTR(STR_STREAM_ID_IN_USE));
      return 1;
    }
    /* If there exists a stream with the same rate already,
     * synchronize this one to that one. */
    if (g_stream_table[index].stream_id != 0 &&
        g_stream_table[index].rate == rate) {
      time_remaining = g_stream_table[index].time_remaining;
    }
  }

  for (index = 0; index < NUM_STREAM_TABLE; index++) {
    if (g_stream_table[index].stream_id == 0) { break; };
  }

  if (index == NUM_STREAM_TABLE) {
    strcpy_P(output, PSTR(STR_STREAM_TABLE_FULL));
    return 1;
  }

  struct stream_struct* item = &g_stream_table[index];

  if (strlen(extra_args + parsed) > 1) {
    // Look for an extra args slot.
    uint8_t i = 0;
    for (; i < NUM_STREAM_EXTRA_ARGS; i++) {
      if (g_stream_args[i][0] == 0) {
        // Found a free one.
        strcpy(g_stream_args[i], extra_args + parsed);
        item->extra_args = g_stream_args[i];
        break;
      }
    }
    if (i == NUM_STREAM_EXTRA_ARGS) {
      strcpy_P(output, PSTR(" not enough extra args slots"));
      return 1;
    }
  } else {
    // These can all share the same null one.
    item->extra_args = g_null_args;
  }

  item->stream_id = stream_id;
  item->cmd_index = message_index;

  item->rate = rate;
  item->time_remaining = time_remaining;
  item->queued_runs = 0;

  strcpy_P(output, PSTR(STR_OK));
  return 0;
}

static uint8_t cmd_ste(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream) {
  uint16_t stream_id;
  char code[3];
  int parsed = 0;
  int len = strlen(extra_args);
  if (sscanf_P(extra_args, PSTR(" %x %c%c%c%n"),
               &stream_id, &code[0], &code[1], &code[2], &parsed) != 4 ||
      parsed != len) {
    strcpy_P(output, PSTR(STR_PARSE_ERROR));
    return 1;
  }

  if (stream_id == 0) {
    strcpy_P(output, PSTR(STR_ID_ZERO));
    return 0;
  }

  for (uint8_t i = 0; i < NUM_STREAM_TABLE; i++) {
    if (g_stream_table[i].stream_id == stream_id) {
      const struct fw_command_struct* fw_command =
          &fw_command_table[g_stream_table[i].cmd_index];
      if (strncmp_P(code, fw_command->code, 3) != 0) {
        strcpy_P(output, PSTR(STR_CODE_MISMATCH));
        return 1;
      }

      g_stream_table[i].stream_id = 0;
      /* De-allocate the extra args.  If this was pointing to the
       * shared empty args, it will be a no-op. */
      g_stream_table[i].extra_args[0] = 0;
      strcpy_P(output, PSTR(STR_OK));
      return 0;
    }
  }

  strcpy_P(output, PSTR(STR_STREAM_NOT_FOUND));
  return 1;
}

static uint16_t stream_list_bitmask(void) {
  uint16_t flags = 0;
  for (uint8_t i = 0; i < NUM_STREAM_TABLE; i++) {
    if (g_stream_table[i].stream_id == 0) { continue; }
    flags |= (1 << g_stream_table[i].stream_id);
  }

  return flags;
}

static uint8_t cmd_stl(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream) {
  *output++ = ' ';
  uint16_to_hex(&output, stream_list_bitmask());
  *output = 0;
  return 0;
}

static uint8_t cmd_stg(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream) {
  uint8_t stream_id = 0;
  int len = strlen(extra_args);
  int parsed = 0;
  if (sscanf_P(extra_args, PSTR(" %x%n"),
               &stream_id, &parsed) != 1 ||
      parsed != len) {
    strcpy_P(output, PSTR(STR_PARSE_ERROR));
  }

  if (stream_id == 0) {
    strcpy_P(output, PSTR(STR_ID_ZERO));
    return 1;
  }

  for (uint8_t i = 0; i < NUM_STREAM_TABLE; i++) {
    if (g_stream_table[i].stream_id != stream_id) { continue; }


    // We found the right one.
    *output++ = ' ';
    uint8_to_hex(&output, stream_id);
    *output++ = ' ';
    uint16_to_hex(&output, g_stream_table[i].rate);
    *output++ = ' ';
    strcpy_P(output, fw_command_table[g_stream_table[i].cmd_index].code);
    output += 3;
    strcpy(output, g_stream_table[i].extra_args);

    return 0;
  }

  strcpy_P(output, PSTR(STR_STREAM_NOT_FOUND));
  return 1;
}

/* static void stream_stop_all(void) { */
/*   for (uint8_t i = 0; i < NUM_STREAM_TABLE; i++) { */
/*     g_stream_table[i].stream_id = 0; */
/*     if (g_stream_table[i].extra_args) { */
/*       g_stream_table[i].extra_args[0] = 0; */
/*     } */
/*   } */
/* } */

static void stream_timer_update(void) {
  for (uint8_t i = 0; i < NUM_STREAM_TABLE; i++) {
    if (!g_stream_table[i].stream_id) { continue; }

    g_stream_table[i].time_remaining--;
    if (!g_stream_table[i].time_remaining) {
      g_stream_table[i].queued_runs++;
      g_stream_table[i].time_remaining = g_stream_table[i].rate;
    }
  }
}

// Run at most one waiting stream operation.
static void stream_poll(void) {
  char output[64];
  output[0] = 0;
  for (uint8_t i = 0; i < NUM_STREAM_TABLE; i++) {
    if (!g_stream_table[i].stream_id) { continue; }

    if (g_stream_table[i].queued_runs) {
      g_stream_table[i].queued_runs--;

      const struct fw_command_struct* fw_command =
          &fw_command_table[g_stream_table[i].cmd_index];
      cmd_func_ptr cmd_func =
          (cmd_func_ptr) pgm_read_word(&fw_command->func);
      char* ptr = output;
      *ptr++ = '!';
      strcpy_P(ptr, PSTR("STM "));
      ptr += 4;
      uint8_to_hex(&ptr, g_stream_table[i].stream_id);
      *ptr++ = ' ';
      strncpy_P(ptr, fw_command->code, 3);
      ptr += 3;
      *ptr = 0;
      uint8_t err = (*cmd_func)(g_stream_table[i].extra_args, ptr,
                                sizeof(output) - (ptr - output), 1);
      if (err == 0xff) {
        // Nothing to do.
        return;
      } else if (err != 0) {
        // Yikes, error!  Report the error and stop the stream.
        g_stream_table[i].stream_id = 0;
        g_stream_table[i].extra_args[0] = 0;
      }

      strcat_P(output, PSTR(EOL));

      software_uart_write_string(output);
      return;
    }
  }
}

#define MAGIC_OVERRUN_CODE 0x04

static void handle_line(char* line_buf) {
  char output[32];
  char *ptr = output;
  *ptr++ = '<';

  uint8_t len = strlen((const char*) line_buf);
  if (len == 0) { return; }
  if (line_buf[0] == MAGIC_OVERRUN_CODE) {
    strcpy_P(ptr, PSTR("ERR overrun"));
  } else {
    // Try to parse the command.
    if (len < 3) {
      strcpy_P(ptr, PSTR("ERR short cmd"));
    } else if (len >= 4 && line_buf[3] != ' ') {
      strcpy_P(ptr, PSTR("ERR cmd not followed by space"));
    } else {
      int8_t index = find_message_index(line_buf);
      if (index < 0) {
        strcpy_P(ptr, PSTR("ERR" STR_UNKNOWN_CMD " "));
        strncat(ptr, line_buf, 3);
      } else {
        const struct fw_command_struct* fw_command = &fw_command_table[index];
        memcpy(ptr, line_buf, 3);
        ptr += 3;
        *ptr = 0; // so that commands start with an empty string
        cmd_func_ptr cmd_func =
            (cmd_func_ptr) pgm_read_word(&fw_command->func);
        (*cmd_func)(
            &line_buf[3],
            ptr, sizeof(output) - (ptr - output) - 3, 0);
      }
    }
  }

  strcat_P(output, PSTR(EOL));
  software_uart_write_string(output);
}

ISR(PCINT_vect) {
  if ((PINB & 0x01) == 0) {
    uint8_t c = software_uart_read_char();
    g_next_char = c;
  }
}

int main() {
  // Set Clock Scaler to 1x, which should give a 8MHz clock
  CLKPR = 0x80;
  CLKPR = 0x00;

  PORTA = 0x00;
  DDRA = 0xff;
  DDRB = 0xfe;

  // Set up Timer 1 to match compare every 1ms.
  OCR1A = 125;
  TCCR1A = 0x02; // CTC on OCR1A
  TCCR1B = 0x06; // CK / 32 (32 * 125 == 4000)

  char line_buf[32];
  uint8_t line_len = 0;

  PCMSK1 = 0x01; // Set PB0 interrupt enable.
  GIMSK |= (1 << PCIE0); // Enable

  i2c_init();

  sei();
  
  strcpy_P(line_buf, PSTR("!GNR HI " DEV_VERSION EOL));
  software_uart_write_string(line_buf);
  
  for (;;) {
    wdt_reset();
    g_main_loop_count++;
    
    PORTA ^= 0x01;
    cli();
    int16_t this_char = g_next_char;
    sei();
    if (this_char >= 0) {
      uint8_t c = this_char;
      cli();
      g_next_char = -1;
      sei();
      
      if (c == '\r' || c == '\n') {
        line_buf[line_len] = 0;
        handle_line(line_buf);
        line_len = 0;
      } else {
        line_buf[line_len++] = c;

        if ((line_len + 1) >= sizeof(line_buf)) {
          line_buf[0] = MAGIC_OVERRUN_CODE;
          line_len--;
        }
      }
    }

    if (TIFR & (1 << OCF1A)) {
      TIFR |= (1 << OCF1A);
      g_timer++;
      stream_timer_update();
      g_main_loop_count = (uint16_t) (((uint32_t) g_main_loop_count) * 7 / 8);
    }

    stream_poll();
  }
}

void stuff() {
  software_uart_read_char();
}
