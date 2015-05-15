/** Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved. */

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

//                 --------------
//               --|PB5      VCC|--
//     PIEZO-    --|PB3      PB2|-- OD hit LED
//     PIEZO+    --|PB4      PB1|-- signal out
//               --|GND      PB0|--
//                 --------------

int main() {
  // Set Clock Scaler to 1x, which should give a 8MHz clock
  CLKPR = 0x80;
  CLKPR = 0x00;

  PORTB = 0x00;
  DDRB = 1 << 2;

  for (;;) {
    PORTB ^= (1 << 2);
    _delay_ms(400);
  }
}
