/** Copyright 2008-2014 Josh Pieper, jjp@pobox.com. */

#ifndef _COMPACT_SOFTWARE_UART_H_
#define _COMPACT_SOFTWARE_UART_H_

#include <avr/cpufunc.h>

/**
 * Before including this file, define the following:
 *
 * UART_WRITE_PORT, UART_WRITE_BIT
 * UART_READ_PORT, UART_READ_BIT
 *
 * UART_CPU_FREQUENCY
 * UART_BAUD_RATE
 *
 * UART_OUTPUT_INVERTED -- if not defined, then configured to work
 * with an external level converer.
 */


#include "usart_rate_calc.h"

#define UART_BAUD_TICKS ((UART_CPU_FREQUENCY) / (UART_BAUD_RATE) - 8)
#define UART_BAUD_TICKS_HALF ((UART_CPU_FREQUENCY) * 3 / ((UART_BAUD_RATE) * 2) - 8)

static uint8_t software_uart_read_char() {
    uint8_t ret = 0;
    uint8_t cnt;
    while (UART_READ_PORT & ( 1 << UART_READ_BIT ) );
    _delay_loop_1(UART_BAUD_TICKS_HALF / 3);
    for (cnt = 8; cnt; cnt--) {
        ret = ret >> 1;
        if (UART_READ_PORT & (1 << UART_READ_BIT) ) {
            ret |= 0x80;
        }
        _delay_loop_1(UART_BAUD_TICKS / 3);
    }
    return ret;
}

static void software_uart_write_char(uint8_t val) {
    uint8_t cnt;
    // start the start bit
    UART_WRITE_PORT &= ~ (1 << UART_WRITE_BIT);
    
    _delay_loop_1(UART_BAUD_TICKS / 3);

    for (cnt = 8; cnt; cnt--) {
        if (val & 0x01) {
            UART_WRITE_PORT |= ( 1 << UART_WRITE_BIT );
        } else {
            UART_WRITE_PORT &= ~ ( 1 << UART_WRITE_BIT );
        }
        val = val >> 1;
        _delay_loop_1(UART_BAUD_TICKS / 3);
    }

    // do the stop bit
    UART_WRITE_PORT |= ( 1 << UART_WRITE_BIT );
    _delay_loop_1(UART_BAUD_TICKS / 3);
}

static void software_uart_write_string(char * c) {
    while (*c != 0) { software_uart_write_char(*c++); }
}
        

#endif /* _COMPACT_SOFTWARE_UART_H_ */
