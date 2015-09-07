// Copyright 2012-2015 Josh Pieper, Mikhail Afanasyev.  All rights
// reserved.

#ifndef HW_H
#define HW_H

/****
  AVR Pinout

  Odroid I/O shield with ATMEGA644P
  5VCC, 16MHz crystal

  Port A:
   A0 I - MP33926 FB (analog in)
   A1 ? -
   A2 ? -
   A3 ? -
   A4 ? -
   A5 ? -
   A6 ? -
   A7 ? -

  Port B:
   B0 ? -
   B1 ? -
   B2 ? -
   B3 ? -
   B4 ? -
   B5 X - ISP
   B6 X - ISP
   B7 X - ISP

  Port C:
   C0 X - SCL
   C1 X - SDA
   C2 O - MC33926 D1
   C3 ? -
   C4 O - MC33926 EN
   C5 I - MC33926 SF
   C6 O - Laser output
   C7 O - Red indicator LED

  Port D:
   D0 I - Serial from odroid (RX)
   D1 O - Serial to odroid (TX)
   D2 ? - Serial from JSER2 (RX)
   D3 ? - Serial to JSER2 (TX)
   D4 O - Fire motor output (OC1B)
   D5 O - Agitator output (OC1A)
   D6 ? -
   D7 ? -

  Fuses
   HFUSE: 0xd9 0b11011001
     * JTAGEN = 0 (JTAG blocks PORTC)
     (note avrdude sometimes mis-reports this as EFUSE, be careful!)
   LFUSE: 0x67 0b01100111
     * CKSEL = 7 (Full power crystal oscillator)

*/

#include "mlib2/mcommon.h"

// Use 115200 in high-speed mode
#define HWUART_UBRR_VALUE  16
#define HWUART_HIGHSPEED   1
#define HWUART_TX_CONTROL  1
#define HWUART_RX_INTERRUPT 1

#define SERIAL_TX_DDR    BITVAR(DDRD, 1)

#define LASER_EN         BITVAR(PORTC, 6)
#define LED_BLUE         BITVAR(PORTC, 7)
#define LED_GREEN        BITVAR(PORTC, 3)

#define MC33926_EN       BITVAR(PORTC, 4)
#define MC33926_D1       BITVAR(PORTC, 2)
#define MC33926_SF       BITVAR(PINC, 5)

#endif
