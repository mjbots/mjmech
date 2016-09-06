// Copyright 2016 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "hit.h"

uint16_t g_hit_count = 0;

static uint16_t g_hit_debounce = 0;
static uint8_t g_old_pin = 1;

void hit_poll(void) {
  uint8_t cur = PIND & 0x01;
  if (g_old_pin != 0 &&
      cur == 0 &&
      g_hit_debounce == 0) {
    g_hit_count++;
    g_hit_debounce = 1000;
  }
  g_old_pin = cur;
}

void hit_timer_update(void) {
  if (g_hit_debounce) { g_hit_debounce--; }
}
