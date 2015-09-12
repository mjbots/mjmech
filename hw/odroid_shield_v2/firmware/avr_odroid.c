// Copyright 2012-2015 Josh Pieper, Mikhail Afanasyev.  All rights
// reserved.

#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include <mlib2/mcommon.h>
#include <mlib2/hwuart.h>

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "hw.h"

// disconnect hercules, allow serial sending
void enable_tx() {
  SERIAL_TX_DDR = 1;
  hwuart_tx_enable(1);
}

// connect herculex, disable serial sending
void disable_tx() {
  hwuart_tx_enable(0);
  SERIAL_TX_DDR = 0;
}

// We are pretending to be a servo at this addr
#define SERVO_ADDR   99

volatile union rx_union{
  struct rx_val_struct {
    uint8_t size;
    uint8_t pID;
    uint8_t cmd;
    uint8_t cs1;
    uint8_t cs2;
    uint8_t data[16 - 5]; // goes from 0 to (size - 8)
  } val;
  uint8_t rawdata[16];
} rx;

volatile uint8_t rx_len = 7;
uint8_t rx_ff_count = 0;

#define RXS_READY  0  // packet is ready, main loop can touch
#define RXS_RX     1  // receiving data
#define RXS_BAD    2  // bad data, waiting for sync
volatile uint8_t rx_status = RXS_BAD;

HWUART_RX_VECT() {
  uint8_t c = _HWUART_DO_RX();
  if (rx_status == RXS_READY) {
    // main buffer not processed yet.
    return;
  }
  if (c != 0xff) {
    rx_ff_count = 0;
  } else {
    rx_ff_count++;
    if (rx_ff_count >= 2) {
      // got sync
      rx_len = 0;
      rx_status = RXS_RX;
      return;
    }
  };
  if (rx_status != RXS_RX) {
    return;
  }

  if (rx_len >= sizeof(rx.val)) {
    // buffer overflow
    rx_status = RXS_BAD;
    return;
  }
  rx.rawdata[rx_len] = c;
  rx_len++;

  if (rx_len == 2 && c != SERVO_ADDR && c != 0xFE) {
    rx_status = RXS_BAD;
    return;
  }
  if (rx_len >= 5 && ((rx_len + 2) == rx.val.size)) {
    rx_status = RXS_READY;
  }
}

uint8_t servo_status_err = 0;
uint8_t servo_status_det = 0;
uint8_t servo_leds = 0;
uint8_t servo_leds_timeout = 0;
uint8_t servo_fire_time = 0;
uint8_t servo_fire_pwm = 0;
uint8_t servo_agitator_pwm = 0;

enum {
  // servo_status_err
  kSSEInvalidPacket = 0x08,
  // servo_status_det
  kSSDChecksumError = 0x04,
  kSSDUnknownCommand = 0x08,
  kSSDExceedRegRange = 0x10,
  kSSDGarbageDetected = 0x20,
  kSSDMotorOn = 0x40
};

static uint8_t calculate_cs1() {
  uint8_t cs1 = rx.val.size ^ rx.val.pID ^ rx.val.cmd;
  uint8_t datalen = rx.val.size - 7;
  for (uint8_t i=0; i<datalen; i++) {
    cs1 ^= rx.val.data[i];
  }
  return cs1;
}

void send_rx_buffer() {
  rx.val.pID = SERVO_ADDR;
  uint8_t cs1 = calculate_cs1();
  rx.val.cs1 = (cs1 & 0xFE);
  rx.val.cs2 = ((cs1 ^ 0xFF) & 0xFE);

  enable_tx();
  hwuart_tx(0xff); // sync
  hwuart_tx(0xff);
  uint8_t tosend = rx.val.size - 2;
  for (uint8_t i=0; i < tosend; i++) {
    hwuart_tx(rx.rawdata[i]);
  }
  hwuart_tx(0); // junk, just in case
  disable_tx();
}

static void handle_STAT_command() {
  // re-use the rx buffer for tx
  rx.val.size = 9;
  rx.val.cmd = 0x47; // STAT_ACK
  rx.val.data[0] = servo_status_err;
  rx.val.data[1] = servo_status_det;
  send_rx_buffer();
}


static uint8_t verify_rx_checksum() {
  uint8_t cs1 = calculate_cs1();
  return (rx.val.cs1 == (cs1 & 0xFE) &&
          rx.val.cs2 == ((cs1 ^ 0xFF) & 0xFE));
}


// Handle write of 1 byte. Return 1 on error.
static uint8_t handle_ram_write(uint8_t addr, uint8_t val) {
  switch (addr) {
    case 43: {
      servo_status_err = val;
      break;
    }
    case 44: {
      servo_status_det = val;
      break;
    }
    case 53: case 83: { // LED Control
      servo_leds_timeout = 200; // 2 seconds
      servo_leds = val;
      break;
    }
    case 80: { // FIRE TIME
      servo_fire_time = val;
      break;
    }
    case 81: { // FIRE PWM
      servo_fire_pwm = val;
      break;
    }
    case 82: { // AGITATOR PWM
      servo_agitator_pwm = val;
      break;
    }
    case 84: { // invalid index
      return 1;
    }
  }

  // Set MOVING flag right now, so it instantly appears in our status.
  if (servo_fire_pwm && servo_fire_time) {
    servo_status_det |= 0x01; // MOVING flag
  }

  return 0;
}

static void handle_RAMWRITE_cmd() {
  uint8_t i;
  uint8_t datalen = rx.val.size - 7;
  uint8_t addr = rx.val.data[0];

  for (i = 2; i < datalen; i++) {
    if (handle_ram_write(addr, rx.val.data[i])) {
      servo_status_err |= kSSEInvalidPacket;
      servo_status_det |= kSSDExceedRegRange;
    }
    addr++;
  }
}

static uint8_t handle_ram_read(uint8_t addr) {
  switch (addr) {
    case 7: return SERVO_ADDR;
    case 43: return servo_status_err;
    case 44: return servo_status_det;
    case 53: case 83:
      return servo_leds;
    case 80: return servo_fire_time;
    case 81: return servo_fire_pwm;
    case 82: return servo_agitator_pwm;
    default:
      return 0;
  }

}

static void handle_RAMREAD_cmd() {
  // parse the packet
  uint8_t addr = rx.val.data[0];
  uint8_t count = rx.val.data[1];

  // re-use the rx buffer for tx
  if ((count + 2) > sizeof(rx.val.data)) {
    count = sizeof(rx.val.data) - 2;
  }
  rx.val.size = count + 11;
  rx.val.cmd = 0x44; // RAM_READ_ACK
  uint8_t i;
  rx.val.data[0] = addr;
  rx.val.data[1] = count;
  for (i=2; i<count; i++) {
    rx.val.data[i] = handle_ram_read(addr);
    addr++;
  }
  i++;
  rx.val.data[i] = servo_status_err;
  i++;
  rx.val.data[i] = servo_status_det;

  send_rx_buffer();
}

static void apply_servo_values() {
    // Apply LED values
    LED_GREEN = !!(servo_leds & 1); // GREEN bit
    LED_BLUE  = !!(servo_leds & 2); // BLUE bit
    LASER_EN  = !!(servo_leds & 4); // RED bit

    if (servo_fire_pwm || servo_agitator_pwm) {
      MC33926_EN = 1;
    } else {
      MC33926_EN = 0;
    }

    // Apply PWM values
    OCR1B = servo_fire_pwm;
    OCR1A = servo_agitator_pwm;
}


int main(void) {
  INIT_SYSTEM_CLOCK();

  // Set up ports. Enable pullups on all unused inputs.
  DDRC = (1 << 2) | (1 << 4) | (1 << 6) | (1<< 7);
  PORTC = ~DDRC;

  // Do not enable TX for now.
  DDRD = (1<<4) | (1<<5);
  // No pullups on serial RX, pullups on all other inputs
  PORTD = ~(DDRD | (1<<0));

  // set up timer/counter 0 to overflow every ~10mS (in CTC mode)
  TCCR0A = (1<<WGM01);
  TIMSK0 = 0; // no interrupts, will check flag by hand.
#if (F_CPU == 16000000)
  TCCR0B = (1<<CS02)|(1<<CS00);  // Prescaler 1/1024
  OCR0A = 156; // 10.048 mS interval
#else
  #error Unsupported CPU speed
#endif

  // set up t/c 1 as phase-correct 8-bit non-inverted PWM on both channels
  // prescaler is 4, which works out to 4kHz  @ 16 MHz clock
  // The old motor boards could only handle up to 10kHz PWM.  The new
  // ones should be capable of up to 20kHz, but I don't see a need to
  // push it yet.
  OCR1A = 0;
  OCR1B = 0;
  TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM10);
  TCCR1B = (1<<CS11);

  hwuart_init();
  hwuart_connect_stdout();

  disable_tx();

  // Enable the MC33926
  MC33926_EN = 1;
  MC33926_D1 = 0;

  sei();
  uint16_t step = 0;

  while (1) {
    if (rx_status == RXS_READY) {
      if (!verify_rx_checksum()) {
        servo_status_err |= kSSEInvalidPacket;
        servo_status_det |= kSSDChecksumError;
      } else if (rx.val.cmd == 0x09) { // REBOOT
        servo_status_err = 0;
        servo_status_det = 0;
        servo_leds = 0;
        servo_fire_time = 0;
        servo_fire_pwm = 0;
        servo_agitator_pwm = 0;
        apply_servo_values();
      } else if (rx.val.cmd == 0x07) { // STAT
        handle_STAT_command();
      } else if (rx.val.cmd == 0x03 &&
                 rx.val.size >= 8) { // RAM WRITE + at least 1 byte
        handle_RAMWRITE_cmd();
        // Do not apply servo values now (to maintain uniform interval)
      } else if (rx.val.cmd == 0x04 &&
                 rx.val.size == 9) { // RAM READ + 2 data bytes
        handle_RAMREAD_cmd();
      } else if (rx.val.cmd == 0x05 || // I_JOG
                 rx.val.cmd == 0x06 || // S_JOG
                 rx.val.cmd == 0x08) { // ROLLBACK
        ; // silently ignore
      } else {
        servo_status_err |= kSSEInvalidPacket;
        servo_status_det |= kSSDUnknownCommand;
      }
      rx_status = RXS_BAD;
    };

    if (!(TIFR0 & (1<<OCF0A))) {
      // Timer not elapsed yet. Spin more.
      continue;
    }
    TIFR0 = (1<<OCF0A);
    // ~10mS has elapsed.

    // expire LEDs
    if (servo_leds_timeout) {
      servo_leds_timeout--;
    } else {
      servo_leds = 0;
      servo_agitator_pwm = 0;
    }

    apply_servo_values();

    // expire fire timer
    if (servo_fire_time) {
      servo_fire_time--;
    } else {
      servo_fire_pwm = 0;
      servo_status_det &= ~0x01; // MOVING flag
    }

    step++;
    // blink LED only if it is not explicilty set
    // turn it on for 100mS every 10 seconds
    if (!servo_leds_timeout) {
      if (step == 10) {
        LED_BLUE = 0;
      } else if (step >= 1000) {
        LED_BLUE = 1;
        step = 0;
      }
    }
    /*
      if ((i % 64) == 2) {
      enable_tx();
      printf_PSTR("SERIAL TEST %d ST=%d LC=%d LEN=%d CMD=%d\n",
      i, rx_status, rx_len, rx.val.cmd);
      disable_tx();
      }
    */
  }
}
