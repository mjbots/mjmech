// Copyright 2014 Josh Pieper.  All rights reserved.

#ifndef HW_H
#define HW_H

/****
  AVR AEG Pinout

  Teensy 2.0++ AT90USB1286

  Port A:
   A0 I -
   A1 I -
   A2 I -
   A3 I -
   A4 I -
   A5 I -
   A6 I -
   A7 I -

  Port B:
   B0 I -
   B1 I -
   B2 I -
   B3 I -
   B4 I -
   B5 I -
   B6 I -
   B7 I -

  Port C:
   C0 O -
   C1 O -
   C2 O -
   C3 O -
   C4 O -
   C5 O -
   C6 O -
   C7 O -

  Port D:
   D0 X - I2C SCL
   D1 X - I2C SDA
   D2 I -
   D3 I -
   D4 O -
   D5 O -
   D6 O -

  Port F:
   F0 I -
   F1 I -
   F2 I -
   F3 I -
   F4 I -
   F5 I -
   F6 I -
   F7 I -

****/

#define HW_JOIN(x, y) x ## y
#define PORT(x) HW_JOIN(PORT, x)
#define PIN(x) HW_JOIN(PIN, x)
#define DDR(x) HW_JOIN(DDR, x)

#endif
