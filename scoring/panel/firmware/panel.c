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

// ADC offset (self-calibrating).
int8_t adc_offset = 0;

// Upper and lower trigger limits.
// These are calculated from adc_offset value.
int8_t adc_trigger_high = 0x7F;
int8_t adc_trigger_low = -0x7F;

// Was hit detected?
uint8_t hit_detected = 0;

// History of average ADC over 256 sample window, used to
// calculate adc_offset.
int8_t adc_offset_h1 = 0;
int8_t adc_offset_h2 = 0;
int8_t adc_offset_h3 = 0;

int8_t trigger_threshold = V_THRESH;

static void wait_for_hit(void) {
  // when adc_last is equal to value below, we report hit as
  // detected and dump data to serial port.
  uint16_t adc_dump_pos = 0;

  // Setup ADC in differential mode: (+)PB4, (-)PB3
  // If REFS2 is present, use 2.56v ref, else uses 1.1v ref
  ADMUX = (1<<REFS1) | (1<<ADLAR) | (1<<MUX2) | (1<<MUX1);

  // Set up in auto mode (conversion per 13.5 cycles), no interrupt.
  // ADPS value to sample rate mapping: 2+1->9.2KHz, 2+0->37KHz, 1+0->74KHz
  ADCSRA = (1<<ADEN)|(1<<ADATE)|(1<<ADPS0)|(1<<ADPS1);
  ADCSRB = (1<<BIN); // bipolar, free-running mode
  ADCSRA |= (1<<ADSC); // start conversion

#define WAIT_ADC_SAMPLE() do { \
    while ((ADCSRA & (1<<ADIF)) == 0) {};  ADCSRA |= (1<<ADIF); } while (0);

  uint8_t adc_data_rotated = 0;

  uint8_t counter_lsb = 0;
  uint8_t counter_msb = 0;

  int16_t raw_val_sum = 0;
  int8_t prev_raw_avg_val = 0;

  hit_detected = 0;

  while (1) {
    // Below is a very tight loop. At a maximum ADC speed, we only have
    // 27 CPU cycles between two readings.
    // Wait until ADC is ready.
    WAIT_ADC_SAMPLE();

    // read the value. We only care about upper 8 bits.
    int8_t val = ADCH;

    raw_val_sum += val;
    // Handle sample counter
    if (++counter_lsb == 0) {
      // Got 256 samples.
      ++counter_msb;
      if (counter_msb > 4 && !hit_detected) {
        // We have been running for a while. Use average ADC value over last
        // 256 samples for calibration. Note that since it is possible that we
        // are going to have a hit very soon, we use previous average value.
        // No danger of overflow here, as types should be auto-promoted to int.
        // We do not update calibration once we are triggered, as we want to
        // preserve original adc_offset value.
        int8_t new_adc_offset = (adc_offset_h1 + adc_offset_h2 +
                                 adc_offset_h3 + prev_raw_avg_val) / 4;
        adc_offset_h1 = adc_offset_h2;
        adc_offset_h2 = adc_offset_h3;
        adc_offset_h3 = prev_raw_avg_val;

        if (counter_msb > 12) {
          // We have enough averages. Update adc_offset.
          adc_offset = new_adc_offset;
          // Update adc_trigger value. This enables triggering.
          adc_trigger_high = adc_offset + trigger_threshold;
          adc_trigger_low = adc_offset - trigger_threshold;
        }
      }

      // Update previous average, reset the sum.
      prev_raw_avg_val = raw_val_sum >> 8;
      raw_val_sum = 0;
    }

    // advance position, store value
    if (++adc_last >= ADC_DATA_COUNT) {
      adc_last = 0;
      adc_data_rotated++;
#ifdef RETURN_DATA_WHEN_IDLE
      // Testing: force trigger once in a while.
      if (adc_data_rotated == 0) { return; }
#endif
    }

    adc_data[adc_last] = val;

    if (hit_detected) {
      // Hit was detected recently. Stop once we get enough samples.
      if (adc_dump_pos == adc_last) {
        return;
      }
    } else if ((val > adc_trigger_high) || (val < adc_trigger_low)) {
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
  // Also, the piezo may have residual mechanical energy from beep.
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
  uint8_t VV = 0x00 | (ADCSRA & 7);
  if (!hit_detected) {
    VV |= (1<<5);
  }
  data_send(VV);
  data_send(trigger_threshold);
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
