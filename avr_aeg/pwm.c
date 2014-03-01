// Copyright 2014 Josh Pieper.  All rights reserved.

#include <avr/io.h>
#include <string.h>

#include "pwm.h"

static uint16_t g_time_remaining[3];

void pwm_init(void) {
  // Set up Timer 1 for PWM, phase correct, TOP=ICR1, frequency = 8kHz
  ICR1 = 1000;
  OCR1A = 0;
  TCCR1A = (1 << COM1A1) | (0 << COM1A0) |
           (1 << COM1B1) | (0 << COM1B0) |
           (1 << COM1C1) | (0 << COM1C0) |
           (1 << WGM11) | (0 << WGM10);
  TCCR1B = (1 << WGM13) | (0 << WGM12) |
           (0 << CS12) | (0 << CS11) | (1 << CS10);

  memset(&g_time_remaining, 0, sizeof(g_time_remaining));
}

void pwm_start(uint8_t channel, uint16_t value, uint16_t time_ms) {
  if (channel >= 3) { return; }

  g_time_remaining[channel] = time_ms;

  if (channel == 0) {
    OCR1A = value;
  } else if (channel == 1) {
    OCR1B = value;
  } else if (channel == 2) {
    OCR1C = value;
  }
}

void pwm_timer_update(void) {
  if (g_time_remaining[0] != 0) {
    g_time_remaining[0]--;
    if (g_time_remaining[0] == 0) { OCR1A = 0; }
  }

  if (g_time_remaining[1] != 0) {
    g_time_remaining[1]--;
    if (g_time_remaining[1] == 0) { OCR1B = 0; }
  }

  if (g_time_remaining[2] != 0) {
    g_time_remaining[2]--;
    if (g_time_remaining[2] == 0) { OCR1C = 0; }
  }
}
