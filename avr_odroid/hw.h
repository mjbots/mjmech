#ifndef HW_H
#define HW_H

/****
  AVR Pinout

  Odroid I/O shield with ATMEGA328
  5VCC, 16MHz crystal
  (x) is a pin number of JP5 connector

  Port B:
   B0 ? D8(6)  -
   B1 ? D9(5)  -
   B2 ? D10    -
   B3 ? D11    -
   B4 ? D12(2) -
   B5 O D13(1) - Blue LED

  Port C:
   C0 ? A0(12) -
   C1 ? A1(11) -
   C2 ? A2     -
   C3 ? A3     -
   C4 X A4(6)  - SDA
   C5 X A5(7)  - SCL

  Port D:
   D0 I D0(20) - Serial from odroid, serial mux to servo
   D1 O D1(19) - Serial to odroid, serial mux from servo
   D2 O D2(18) - /Serial mux enable (0 to connect serial to servo)
   D3 ? D3     - connected to servo pin header
   D4 O D4(16) - LASER FET
   D5 ? D5     - connected to servo pin header
   D6 O D6(14) - mixer output (hooked to green LED)
   D7 O D7(13) - fire output

*/

#include "mlib2/mcommon.h"

// Use 115200 in high-speed mode
#define HWUART_UBRR_VALUE  16
#define HWUART_HIGHSPEED   1
#define HWUART_TX_CONTROL  1
#define HWUART_RX_INTERRUPT 1

#define TX_BYPASS_nEN    BITVAR(PORTD, 2)
#define SERIAL_TX_DDR    BITVAR(DDRD, 1)

#define LASER_EN         BITVAR(PORTD, 4)
#define LED_BLUE         BITVAR(PORTB, 5)

#define MIXER_EN         BITVAR(PORTD, 6)
#define FIRE_MOTOR_EN    BITVAR(PORTD, 7)

#endif
