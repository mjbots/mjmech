// Copyright 2012 Josh Pieper.  All rights reserved.

#include "util.h"

static void nyb_to_hex(char** buf, uint8_t value) {
  if (value < 10) {
    **buf = value + '0';
  } else {
    **buf = value + 'A' - 10;
  }
  (*buf)++;
}

void uint8_to_hex(char** buf, uint8_t value) {
  nyb_to_hex(buf, (value & 0xf0) >> 4);
  nyb_to_hex(buf, (value & 0x0f));
}

void uint16_to_hex(char** buf, uint16_t value) {
  nyb_to_hex(buf, (value & 0xf000) >> 12);
  nyb_to_hex(buf, (value & 0x0f00) >> 8);
  nyb_to_hex(buf, (value & 0x00f0) >> 4);
  nyb_to_hex(buf, (value & 0x000f));
}

/** Parse 2 hex digits from the given string and stores them into
 * *val.  In unparseable, return non-zero. */
uint8_t parse_hex2(char**arg, uint8_t* val) {
  uint8_t v;
  *val = 0;
  for (v = 0; v < 2; v++) {
    // Skip some characters.
    while (**arg == ' ' || **arg == ':') {
      (*arg)++;
    }
    if (**arg >= '0' && **arg <= '9') {
      *val = (*val << 4) | (**arg - '0');
    } else if (**arg >= 'a' && **arg <= 'f') {
      *val = (*val << 4) | (**arg - 'a' + 10);
    } else if (**arg >= 'A' && **arg <= 'F') {
      *val = (*val << 4) | (**arg - 'A' + 10);
    } else {
      return 1;
    }
    (*arg)++;
  }
  return 0;
}
