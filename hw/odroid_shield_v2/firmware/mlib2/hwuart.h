#ifndef __MLIB2_HWUART_H__
#define __MLIB2_HWUART_H__

#include <mlib2/mcommon.h>

// User-provided include file which should be used to configure library.
#include "hw.h"

//
//         USAGE
//
// must define in hw.h:
//    HWUART_UBRR_VALUE  - raw UBRR value to set serial speed
//    HWHWUART_HIGHSPEED   - either -1 (normal speed) or 1 (high speed)
//
// You can optionally define settings to non-zero values:
//    HWUART_TX_CONTROL
//       Extra TX functions such as enable/disable
//       If this is defined, TX is initialized disabled and must be enable
//       explicitly.
//    HWUART_RX_INTERRUPT
//    HWUART_UDRE_INTERRUPT
//       Enable RX complete/TX data regiter empty interrupts
//
// (No zeros are used because preprocessor compares all undefined symbols as zero)

// Init UART module. Will enable RX and TX unless HWUART_NO_INIT_TX is defined.
void hwuart_init();

// Send a character. Will block if buffer is full and cannot transmit.
void hwuart_tx(char c);

// Check transmit buffer state.
// Return 1 if hwuart_tx will block because the buffer is full.
uint8_t hwuart_tx_is_full();

#if (HWUART_TX_CONTROL)
// Disable/enable TX; depending on the usage, you might want to also
// tristate pin.
void hwuart_tx_enable(uint8_t enable);

// Return 1 if tx is enabled.
uint8_t hwuart_tx_is_enabled();

// Return 1 if tx is transmitting. Once this goes to zero,
// the transmitter may be disabled safely.
uint8_t hwuart_tx_is_busy();

#else
static inline uint8_t hwuart_tx_is_enabled() { return 1; }
#endif

// Assigns 'stdout' stream to hwuart.
void hwuart_connect_stdout();

// non block; 0 if no data
//  warning: slow if RX buffer is enabled and has many characters
uint8_t hwuart_recv();


#if (HWUART_UBRR_VALUE <= 0)
#warning Invalid baudrate requested: UBRR too low
#endif

#if (HWUART_UBRR_VALUE >= 1024)
#error Invalid baudrate requested: UBRR too high
#endif


// for two-UART micros, use UART zero
#if defined (__AVR_ATmega88__)  || \
  defined (__AVR_ATmega168__)  ||  \
  defined (__AVR_ATmega328P__) ||  \
  defined (__AVR_ATmega644P__) ||  \
  defined (__AVR_ATmega1281__) ||  \
  defined (__AVR_ATmega128__)
# define HWUART_ZERO
#endif


#if defined(__AVR_ATmega163__)
# define HWUART_RX_VECT()    SIGNAL(UART_RX_vect)
# define HWUART_UDRE_VECT()  SIGNAL(UART_UDRE_vect)
#elif defined(__AVR_ATmega128__) || defined(__AVR_ATmega644P__)
# define HWUART_RX_VECT()    SIGNAL(USART0_RX_vect)
# define HWUART_UDRE_VECT()  SIGNAL(USART0_UDRE_vect)
#elif defined(__AVR_ATmega8__)
# define HWUART_RX_VECT()    SIGNAL(USART_RXC_vect)
# define HWUART_UDRE_VECT()  SIGNAL(USART_UDRE_vect)
#else
# define HWUART_RX_VECT()    SIGNAL(USART_RX_vect)
# define HWUART_UDRE_VECT()  SIGNAL(USART_UDRE_vect)
#endif

#if !(HWUART_UDRE_INTERRUPT)
#undef HWUART_UDRE_VECT
#endif

#if !(HWUART_RX_INTERRUPT)
#undef HWUART_RX_VECT
#endif


#ifdef HWUART_ZERO
# define UCSRx(n)   UCSR0 ## n
# define UBx(n)     n ## 0
# define UDRx       UDR0
#else
# define UCSRx(n)   UCSR ## n
# define UBx(n)   n
# define UDRx       UDR
#endif

// below are macro accessors which should not be normally used.

// true if port has char
#define _HWUART_CAN_RX()     ((UCSRx(A) & (1<<UBx(RXC))))
// read char. call at most once
#define _HWUART_DO_RX()      (UDRx)

// true if can TX (safe to call DO_TX)
#define _HWUART_TX_EMPTY()    (UCSRx(A) & (1<<UBx(UDRE)))
#define _HWUART_DO_TX(c)      UDRx = (c);
// true if transmit was complete (inactive if interrupt is enabled)
// cannot be set other than by completing TX of a char.
#define _HWUART_TX_COMPLETE() (UCSRx(A) & (1<<UBx(TXC)))
// clear TX COMPLETE flag (note special bit function)
#define _HWUART_TX_COMPLETE_CLEAR()  UCSRx(A) |= (1<<UBx(TXX)

#endif
