/*
Copyright (C) 2010-2014 Mikhail Afanasyev

This file is part of AVR-mlib2.

AVR-mlib is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

AVR-mlib is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with AVR-mlib.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/interrupt.h>
#include <string.h>
#include <mlib2/hwuart.h>
#include <stdio.h>

static inline void hwuart_write_UCSRA(uint8_t val) {
  // UCSRA is tricky -- some bits are magic and should
  // be written as zero, while other bits are regular, and should
  // not be modified needlessly.

#if (HWUART_HIGHSPEED == 1)
  UCSRx(A) = val | (1<<UBx(U2X));
#elif (HWUART_HIGHSPEED == -1)
  UCSRx(A) = val;
#else
  #error HWUART_HIGHSPEED is not defined properly
#endif

}

void hwuart_init() {
  // setup UART

#if defined (__AVR_ATmega128__)
  UBRR0H = (HWUART_UBRR_VALUE) >> 8;
  UBRR0L = (HWUART_UBRR_VALUE) ;
#elif defined (__AVR_ATmega8__) ||    \
  defined (__AVR_ATtiny2313__) ||     \
  defined (__AVR_ATtiny4313__)
  UBx(UBRRH) = (HWUART_UBRR_VALUE) >> 8;
  UBx(UBRRL) = (HWUART_UBRR_VALUE) ;
#else
  UBx(UBRR) = HWUART_UBRR_VALUE;
#endif

  UCSRx(B) = (1<<UBx(RXEN))  // transmit
#if !(HWUART_TX_CONTROL)
        | (1<<UBx(TXEN))
#endif
#if (HWUART_RX_INTERRUPT)
        | (1<<UBx(RXCIE))
#endif
#if (HWUART_UDRE_INTERRUPT)
        | (1<<UBx(UDRIE))
#endif
        ;

#ifndef __AVR_ATmega163__
  // TODO: add URSEL here for atmega8
  UCSRx(C) = (1<<UBx(USBS))
#if defined(URSEL)
        | (1 << URSEL) // ugly, ugly
#endif
        | (3<<UBx(UCSZ0));   // 8-N-2
#endif

  hwuart_write_UCSRA(0);
};


uint8_t hwuart_tx_is_full() {
  if (!hwuart_tx_is_enabled()) { return 0; }
  return !_HWUART_TX_EMPTY();
};

#if (HWUART_TX_CONTROL)
static uint8_t hwuart_tx_busy_flag = 0;

void hwuart_tx_enable(uint8_t enable) {
  if (enable) {
    UCSRx(B) |= (1<<UBx(TXEN));
  } else {
    if (hwuart_tx_is_enabled()) {
      // Flush any data in progress.
      while (hwuart_tx_is_busy()) {};
    }
    UCSRx(B) &=~ (1<<UBx(TXEN));
  };
}

uint8_t hwuart_tx_is_enabled() {
  return !!(UCSRx(B) & (1<<UBx(TXEN)));
};

uint8_t hwuart_tx_is_busy() {
  if (!hwuart_tx_is_enabled()) { return 0; }
  // Assume we never have TX complete interrupt
  if (UCSRx(A) & (1<<UBx(TXC))) {
    hwuart_tx_busy_flag = 0;
    // Clear flag.
    hwuart_write_UCSRA(1<<UBx(TXC));
  }
  return hwuart_tx_busy_flag;
}

#endif // HWUART_TX_CONTROL


void hwuart_tx(char c) {
  if (!hwuart_tx_is_enabled()) { return; }
  // wait for transmission to be completed, then TX
  while (!_HWUART_TX_EMPTY()) {};
#ifdef HWUART_TX_CONTROL
  hwuart_tx_busy_flag = 1;
#endif
  _HWUART_DO_TX(c);
};

static int hwuart_stdio_write(char c, FILE* unused) {
  hwuart_tx(c);
  return 0;
};

static FILE hwuart_stdout =
  FDEV_SETUP_STREAM(hwuart_stdio_write, NULL,
                    _FDEV_SETUP_WRITE);

void hwuart_connect_stdout() {
  stdout = &hwuart_stdout;
}


uint8_t hwuart_recv() {
  if (!_HWUART_CAN_RX()) {
    return 0;
  }
  uint8_t rv = _HWUART_DO_RX();
  return rv;
};
