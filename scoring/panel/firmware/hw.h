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
  header: 0xFF 0x36 0xB0 0xD4 LL VV TT ZZ 00 DATA CC
  Where:
    LL - 1 byte, (data length / 4)
      data length includes all following bytes except checksum
    VV - 1 byte, packet version + sample rate.
      bits 7..6: version, set to 0
      bit  5   : set to 1 if this is no-trigger return.
      bits 4..3: reserved
      bits 2..0: sample rate -- lower 3 bits of ADCSRA
    TT - 1 byte, current threshold trigger is 'abs(data) > TT'
    ZZ - 1 byte, ADC offset
    00 - 1 byte, reserved, always zero
    DATA - (rest of bytes) binary samples -- signed 8-bit integers
           the data does NOT have ADC offset subtracted.
    CC - 1 byte, simple sum of all bytes
*/

#define P_SIGNAL   1
#define P_LED      2

// Threshold value -- when the hit is detected (0..255).
#define V_THRESH   30

// Threshold count -- how many samples must exceed the threshold
#define V_HIGH_COUNT  2

// size of data buffer, mostly limited by RAM
#define ADC_DATA_COUNT   400

// Serial line divisor. Defines baudrate.
#define SSERIAL_DIVISOR   138 // 69 = 115200, 138 = 57600

// ADC prescaler
// Number from 1 to 7, ADC will sample at (8e6 / 13.5 / 2**X) Hz
// 5 = 18KHz (max quality), 3 = 74KHz (max recommended by ATMEL), 2=148KHz
#define ADC_PRESCALER     2

// When defined, will register trigger every (256 * V_DATA_COUNT) ADC samples
// even if not above the treshold.
//#define RETURN_DATA_WHEN_IDLE


#endif
