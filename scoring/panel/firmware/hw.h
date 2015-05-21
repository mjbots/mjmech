#ifndef __HW_H__
#define __HW_H__

/*
                 --------------
               --|PB5      VCC|--
     PIEZO-    --|PB3      PB2|-- OD hit LED
     PIEZO+    --|PB4      PB1|-- signal out
               --|GND      PB0|--
                 --------------

signal out is a multi-purpose pin:
 - by default, it is pulled-up
 - it is pulsed to ground for few mS on hit
 - then if enabled, we transmit detailed hit data in serial format.

serial format:
  header: 0xFF 0x36 0xB0 0xD4 LL VV TT ZZ 00 <binary-data> CC
  Where:
    LL - 1 byte, (data length / 4)
      data length includes all following bytes except checksum
    VV - 1 byte, packet version + sample rate.
      bits 7..6: version, set to 0
      bits 5..3: reserved
      bits 2..0: sample rate -- lower 3 bits of ADCSRA
    TT - 1 byte, current threshold
    ZZ - 1 byte, ADC offset
    00 - 1 byte, reserved, always zero
    CC - 1 byte, simple sum of all bytes
*/

#define P_SIGNAL   1
#define P_LED      2

// Threshold value -- when the hit is detected (0..255)
// 12 is good.
#define V_THRESH   12

// size of data buffer, mostly limited by RAM
#define ADC_DATA_COUNT   400

// Serial line divisor. Defines baudrate.
#define  SSERIAL_DIVISOR   138 // 69 = 115200, 138 = 57600

// When defined, will register trigger every (256 * V_DATA_COUNT) ADC samples
// even if not above the treshold.
//#define RETURN_DATA_WHEN_IDLE


#endif
