/** Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved. */

#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>

//                 --------------
//               --|PB0      PA0|-- 
//               --|PB1      PA1|-- 
//               --|PB2      PA2|-- 
//               --|PB3      PA3|-- 
//               --|VCC      GND|--
//               --|GND     AVCC|--
//               --|PB4      PA4|-- 
//               --|PB5      PA5|-- 
//               --|PB6      PA6|-- 
//       Reset   --|PB7      PA7|-- 
//                 --------------

int main() {
  // Set Clock Scaler to 1x, which should give a 8MHz clock
  CLKPR = 0x80;
  CLKPR = 0x00;

  PORTA = 0x00;
  DDRA = 0xff;

  for (;;) {
    PORTA ^= 0x01;
    _delay_ms(1000);
  }
}
