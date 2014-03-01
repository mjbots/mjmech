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
