// Copyright 2013 Josh Pieper, jjp@pobox.com.  All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

// The following routine is copied from:
// http://www.pjrc.com/teensy/jump_to_bootloader.html

void enter_bootloader(void) {
  cli();
  // disable watchdog, if enabled
  wdt_disable();
  // disable all peripherals
  UDCON = 1;
  USBCON = (1<<FRZCLK);  // disable USB
  UCSR1B = 0;
  _delay_ms(5);
#if defined(__AVR_AT90USB162__)                // Teensy 1.0
  EIMSK = 0; PCICR = 0; SPCR = 0; ACSR = 0; EECR = 0;
  TIMSK0 = 0; TIMSK1 = 0; UCSR1B = 0;
  DDRB = 0; DDRC = 0; DDRD = 0;
  PORTB = 0; PORTC = 0; PORTD = 0;
  asm volatile("jmp 0x3E00");
#elif defined(__AVR_ATmega32U4__)              // Teensy 2.0
  EIMSK = 0; PCICR = 0; SPCR = 0; ACSR = 0; EECR = 0; ADCSRA = 0;
  TIMSK0 = 0; TIMSK1 = 0; TIMSK3 = 0; TIMSK4 = 0; UCSR1B = 0; TWCR = 0;
  DDRB = 0; DDRC = 0; DDRD = 0; DDRE = 0; DDRF = 0; TWCR = 0;
  PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0; PORTF = 0;
  asm volatile("jmp 0x7E00");
#elif defined(__AVR_AT90USB646__)              // Teensy++ 1.0
  EIMSK = 0; PCICR = 0; SPCR = 0; ACSR = 0; EECR = 0; ADCSRA = 0;
  TIMSK0 = 0; TIMSK1 = 0; TIMSK2 = 0; TIMSK3 = 0; UCSR1B = 0; TWCR = 0;
  DDRA = 0; DDRB = 0; DDRC = 0; DDRD = 0; DDRE = 0; DDRF = 0;
  PORTA = 0; PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0; PORTF = 0;
  asm volatile("jmp 0xFC00");
#elif defined(__AVR_AT90USB1286__)             // Teensy++ 2.0
  EIMSK = 0; PCICR = 0; SPCR = 0; ACSR = 0; EECR = 0; ADCSRA = 0;
  TIMSK0 = 0; TIMSK1 = 0; TIMSK2 = 0; TIMSK3 = 0; UCSR1B = 0; TWCR = 0;
  DDRA = 0; DDRB = 0; DDRC = 0; DDRD = 0; DDRE = 0; DDRF = 0;
  PORTA = 0; PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0; PORTF = 0;
#if defined(ATMEL_DFU)
  asm volatile("jmp 0x1e000");
#else
  asm volatile("jmp 0x1FC00");
#endif
#endif
}
