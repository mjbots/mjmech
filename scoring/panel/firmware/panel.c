#include <avr/eeprom.h>
#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/atomic.h>

#include <ctype.h>

#include "hw.h"

// Data buffer
// We always update it, no matter if hit is detected or not.
uint8_t adc_data[ADC_DATA_COUNT];

// Last index written to.
uint16_t adc_last = 0;

// ADC offset
int8_t adc_offset = 0;

static void wait_for_hit(void) {
  // Was hit detected?
  uint8_t hit_detected = 0;

  // when adc_last is equal to value below, we report hit as
  // detected and dump data to serial port.
  uint16_t adc_dump_pos = 0;

  // Setup ADC in differential mode with both (+) and (-) at PB4 for offset
  // calibration.
  // If REFS2 is present, use 2.56v ref, else uses 1.1v ref
  ADMUX = (1<<REFS1) | (1<<ADLAR) | (1<<MUX2);

  // Set up in auto mode (conversion per 13.5 cycles), no interrupt.
  // ADPS value to sample rate mapping: 2+1->9.2KHz, 2+0->37KHz, 1+1->74KHz
  ADCSRA = (1<<ADEN)|(1<<ADATE)|(1<<ADPS2)|(1<<ADPS1);
  ADCSRB = (1<<BIN); // bipolar, free-running mode
  ADCSRA |= (1<<ADSC); // start conversion

#define WAIT_ADC_SAMPLE() do { \
    while ((ADCSRA & (1<<ADIF)) == 0) {};  ADCSRA |= (1<<ADIF); } while (0);

  int16_t offset_sum = 0;
  // Read 256 samples for calibration
  WAIT_ADC_SAMPLE();
  for (uint16_t count=0; count<0x100; count++) {
    offset_sum += ADCH;
    WAIT_ADC_SAMPLE();
  }
  adc_offset = offset_sum >> 8;

  // Setup ADC in differential mode: (+)PB4, (-)PB3
  ADMUX = (1<<REFS1) | (1<<ADLAR) | (1<<MUX2) | (1<<MUX1);
  uint8_t led_counter = 0;
  // Give ADC time to switch.
  WAIT_ADC_SAMPLE();
  WAIT_ADC_SAMPLE();

  while (1) {
    // Below is a very tight loop. At a maximum ADC speed, we only have
    // 27 CPU cycles between two readings.
    // Wait until ADC is ready.
    WAIT_ADC_SAMPLE();

    // read the value. We only care about upper 8 bits.
    int8_t val = ADCH - adc_offset;

    // advance position, store value
    if (++adc_last >= ADC_DATA_COUNT) {
      adc_last = 0;
      // blink LED for a short time every time led_counter overflows.
      if (++led_counter <= 3) {
        //PORTB ^= (1<<P_LED);
      }
      // Testing: force trigger once in a while.
      if (led_counter == 0) { return; }
    }
    adc_data[adc_last] = val;

    if (hit_detected) {
      // Hit was detected recently. Stop once we get enough samples.
      if (adc_dump_pos == adc_last) {
        return;
      }
    } else if (val > V_THRESH) {
      // Just detected a hit!
      hit_detected = 1;
      // Set the dump time. We tune it such that 25% of buffer
      // is before the trigger, and the resit is after the trigger.
      adc_dump_pos = adc_last + ADC_DATA_COUNT - (ADC_DATA_COUNT / 4);
      while (adc_dump_pos >= ADC_DATA_COUNT) {
        adc_dump_pos -= ADC_DATA_COUNT;
      }
    }
  }

#undef WAIT_ADC_SAMPLE
}

void do_beep() {
  // Play a beeping sound
  // Stop ADC
  ADCSRA = 0;

  // Set port to output.
  DDRB |= (1<<3)|(1<<4);
  PORTB |= (1<<3);

  for (uint16_t i=0; i<200; i++) {
    PORTB ^= (1<<3)|(1<<4);
    _delay_us(500);
  }
  PORTB &=~ ((1<<3)|(1<<4));
  DDRB &=~ ((1<<3)|(1<<4));
  // ADC was running, so it may be sampling something stupid now.
  // Also, the piezo may have residual mechanical energey from beep.
  // Sleep for a while to let it settle.
  _delay_ms(10);
}

//
//        SERIAL STUFF START
//

void softserial_init() {
  // no timer interrupt, but we will check the flag
  // this requires
  TCCR0A = (1<<WGM01); // CTC
  TCCR0B = (1<<CS00); // prescaler:/1
  OCR0A = SSERIAL_DIVISOR;
};


// this is a fast send function which does not do interrupts
// it does, however, uses timer0 to measure time intervals
void softserial_send(char c) {
#define WAIT_BIT()  { while (!(TIFR & (1<<OCF0A))) {}; TIFR = (1<<OCF0A); };
#define TX_MARK()    PORTB |= (1<<P_SIGNAL)
#define TX_SPACE()   PORTB &=~ (1<<P_SIGNAL)
  // The block below is always ran without interrupts
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // Set pin to output
    TX_MARK();
    DDRB |= (1<<P_SIGNAL);

    WAIT_BIT(); WAIT_BIT();
    TX_SPACE(); WAIT_BIT(); // START bit
    for (uint8_t bc=0; bc<8; bc++) {
      // send bit
      if (c & 1) { TX_MARK(); } else { TX_SPACE(); };
      c = c >> 1;
      WAIT_BIT();
    };
    TX_MARK(); WAIT_BIT(); // STOP bit
    WAIT_BIT(); // another STOP

    // Release the pin
    DDRB &= ~(1<<P_SIGNAL);
  }
#undef WAIT_BIT
#undef TX_MARK
#undef TX_SPACE
};

uint8_t tx_checksum;
void data_send(uint8_t data) {
  tx_checksum += data;
  softserial_send(data);
}

void send_info() {
  // Transmit buffer in proper format
  tx_checksum = 0;
  data_send(0xFF);
  data_send(0x36);
  data_send(0xB0);
  data_send(0xD4);
  data_send((ADC_DATA_COUNT + 4)/ 4);
  data_send(0x00 | (ADCSRA & 7));
  data_send(V_THRESH);
  data_send(adc_offset);
  data_send(0x00);
  uint16_t pos = adc_last;
  do {
    pos++;
    if (pos >= ADC_DATA_COUNT) { pos = 0; }
    data_send(adc_data[pos]);
  } while (pos != adc_last);
  softserial_send(tx_checksum);
}

int main() {
  // Set Clock Scaler to 1x, which should give a 8MHz clock
  CLKPR = 0x80;
  CLKPR = 0x00;

  DDRB = (1<<P_LED);
  PORTB = (1<<P_SIGNAL)|(1<<P_LED);

  softserial_init();

  // No interrupts anywhere.
  cli();

  while (1) {
    wait_for_hit();

    // Set LED for a while
    PORTB &=~ (1<<P_LED);
    //do_beep();
    send_info();
    for (uint16_t i=0; i<400; i++) {
      _delay_ms(1);
    }
    PORTB |= (1<<P_LED);
  }

}
