// Copyright 2012 Josh Pieper, jjp@pobox.com.  All rights reserved.
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
