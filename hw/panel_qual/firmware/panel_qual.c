// Copyright 2012-2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "enter_bootloader.h"
#include "hw.h"
#include "usb_serial.h"
#include "util.h"

#define DEV_VERSION "panel-qual" ARR_DBG_FLAG " v0.1 r" ARR_GIT_REV
#define EOL "\r\n"

#define STR_OK " OK"
#define STR_PARSE_ERROR " could not parse"

static uint16_t g_timer;
static uint16_t g_main_loop_count;

static uint16_t g_arm_code;
static uint16_t g_arm_timer;
#define ARM_TIMEOUT_MS 1000

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
    PORTD ^= 0x80;
  }
  return 0;
}

static uint8_t cmd_bot(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream) {
  wdt_disable();
  enter_bootloader();
  return 0;
}

static uint8_t cmd_tmq(char* extra_args, char* output, uint8_t output_len,
                       uint8_t stream) {
  *output++ = ' ';
  uint16_to_hex(&output, g_timer);
  *output = 0;
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

static struct fw_command_struct PROGMEM fw_command_table[] = {
  { "PNG", cmd_png, 0 },
  { "STB", cmd_stb, 0 },
  { "STE", cmd_ste, 0 },
  { "STL", cmd_stl, (1 << CMD_FLAGS_STREAMABLE) },
  { "STG", cmd_stg, 0 },
  { "VER", cmd_ver, 0 },
  { "RST", cmd_rst, 0 },
  { "BOT", cmd_bot, 0 },
  { "TMQ", cmd_tmq, (1 << CMD_FLAGS_STREAMABLE) },
};

#define NUM_FW_COMMAND (sizeof(fw_command_table) / sizeof(*fw_command_table))

static int8_t find_message_index(char* code) {
  int8_t index = 0;
  for (; index < NUM_FW_COMMAND; index++) {
    struct fw_command_struct* fw_command = &fw_command_table[index];
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
} g_stream_table[12];

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

  struct fw_command_struct* fw_command = &fw_command_table[message_index];
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
      struct fw_command_struct* fw_command =
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

static void stream_stop_all(void) {
  for (uint8_t i = 0; i < NUM_STREAM_TABLE; i++) {
    g_stream_table[i].stream_id = 0;
    if (g_stream_table[i].extra_args) {
      g_stream_table[i].extra_args[0] = 0;
    }
  }
}

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

      struct fw_command_struct* fw_command =
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

      usb_serial_write((uint8_t*) output, strlen(output));
      return;
    }
  }
}

#define MAGIC_OVERRUN_CODE 0x04

static void handle_line(char* line_buf) {
  char output[80];
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
        strcpy_P(ptr, PSTR("ERR" STR_UNKNOWN_CMD));
      } else {
        struct fw_command_struct* fw_command = &fw_command_table[index];
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
  usb_serial_write((const uint8_t*) output, strlen(output));
}

/** Magic to ensure that the watchdog doesn't get us into an infinite
 * loop if it does trip. */
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));

void get_mcusr(void)                            \
    __attribute__((naked))                      \
    __attribute__((section(".init3")));
void get_mcusr(void)
{
  mcusr_mirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

/** The MAIN loop. */
int main(void) {
  // Turn off the CPU prescale.
  CLKPR = 0x80;
  CLKPR = 0x00;

  // PORTA are general purpose inputs.
  DDRA = 0x00;
  PORTA = 0xff; // pull-ups all enabled

  PORTD = 0xf0;
  DDRD = 0xf0;
  DDRF = 0x00; // These are ADC lines.
  PORTF = 0x00;

  usb_init();

  // Set up Timer 0 to match compare every 1ms.
  OCR0A = 250;
  TCCR0A = 0x02; // CTC
  TCCR0B = 0x03; // CK/64 (64 * 250 == 16000)

  wdt_enable(ARR_WDT_PERIOD);

  sei();
  char line_buf[64];
  uint8_t line_len = 0;
  uint8_t usb_ready = 0;
  while (1) {
    wdt_reset();
    g_main_loop_count++;

    if (usb_configured() && (usb_serial_get_control() & USB_SERIAL_DTR)) {
      if (!usb_ready) {
        usb_ready = 1;
        strcpy_P(line_buf, PSTR("!GNR WELCOME " DEV_VERSION EOL));
        usb_serial_write((uint8_t*)line_buf, strlen(line_buf));
        line_len = 0;
      }
    } else {
      stream_stop_all();
      usb_serial_flush_input();
      usb_ready = 0;
      line_len = 0;
      g_arm_code = 0;
    }

    if (usb_serial_available()) {
      int16_t c = usb_serial_getchar();
      if (c == '\r' || c == '\n') {
        line_buf[line_len] = 0;
        handle_line(line_buf);
        if (g_arm_code) { g_arm_timer = ARM_TIMEOUT_MS; }
        line_len = 0;
      } else {
        line_buf[line_len++] = c & 0xff;

        if ((line_len + 1) >= sizeof(line_buf)) {
          /* Clobber the first byte so that this line will be reported
             as an error. */
          line_buf[0] = MAGIC_OVERRUN_CODE;
          line_len--;
        }
      }
    }

    if (TIFR0 & (1 << OCF0A)) {
      TIFR0 |= (1 << OCF0A);
      g_timer++;
      stream_timer_update();

      g_main_loop_count = (uint16_t) (((uint32_t) g_main_loop_count) * 7 / 8);
    }

    stream_poll();
    usb_poll();
  }
}
