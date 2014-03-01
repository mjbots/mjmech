// Copyright 2014 Josh Pieper.  All rights reserved.

#pragma once

/***********************************************************
 * PWM Module
 *
 * Resources:
 *  + Timer1
 *  + N bytes of RAM
 */

/** Initialize the PWM subsystem. */
void pwm_init(void);

/** Start a PWM pulse.
 *
 * @param channel 0-2, for channel A, B, or C
 * @param value 0-1000, tenth of a percent of duty cycle
 * @param time_ms milliseconds to leave pulse on
 */
void pwm_start(uint8_t channel, uint16_t value, uint16_t time_ms);

/** Call every millisecond. */
void pwm_timer_update(void);
