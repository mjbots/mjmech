// Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

#include <avr/interrupt.h>
#include <avr/io.h>

#include "serial.h"

static uint32_t g_baud_rate;
static uint8_t g_char_size;
static uint8_t g_parity;
static uint8_t g_stop_bits;
static uint8_t g_full_duplex;

#define BUFFER_SIZE 64
static uint8_t g_recv_buffer[BUFFER_SIZE];
static uint8_t g_recv_buffer_start, g_recv_buffer_end;

static uint8_t g_send_buffer[BUFFER_SIZE];
static uint8_t g_send_buffer_start, g_send_buffer_end;

static uint8_t g_error;

void serial_init(void) {
  serial_configure(115200, 8, SERIAL_PARITY_NONE, 1, 0);
}

static uint32_t abs32(int32_t value) {
  if (value < 0) { return -value; }
  return value;
}

uint8_t serial_configure(uint32_t baud_rate,
                         uint8_t char_size,
                         uint8_t parity,
                         uint8_t stop_bits,
                         uint8_t full_duplex) {
  if (baud_rate < 300) {
    return SERIAL_ERR_INVALID_BAUD;
  }

  uint16_t maybe_ubrr =
      (uint16_t) ((((uint32_t) F_CPU) / (8 * baud_rate)) - 1);

  uint32_t best_baud_error = UINT32_MAX;
  uint16_t best_ubrr = maybe_ubrr;
  /* Try ubrr values around this one to find the value with the least
   * baud rate error, and see what that error is. */
  for (int8_t i = -1; i <= 1; i++) {
    uint16_t this_ubrr = maybe_ubrr + i;
    uint32_t actual_baud =
        ((uint32_t) F_CPU) / (uint32_t) (8 * (this_ubrr + 1));
    uint32_t error =
        abs32(((int32_t) actual_baud - (int32_t) baud_rate));
    if (error < best_baud_error) {
      best_baud_error = error;
      best_ubrr = this_ubrr;
    }
  }

  uint32_t tenth_percentage_error =
      best_baud_error * ((uint32_t) 1000) / baud_rate;
  if (tenth_percentage_error > 30) {
    return SERIAL_ERR_INVALID_BAUD;
  }

  if (parity != SERIAL_PARITY_NONE &&
      parity != SERIAL_PARITY_EVEN &&
      parity != SERIAL_PARITY_ODD) {
    return SERIAL_ERR_INVALID_PARITY;
  }
  if (char_size < 5 || char_size > 8) {
    return SERIAL_ERR_INVALID_CHAR_SIZE;
  }
  if (stop_bits < 1 || stop_bits > 2) {
    return SERIAL_ERR_INVALID_STOP_BITS;
  }

  UBRR1 = best_ubrr;
  UCSR1A = (1 << U2X1);
  UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1) | (1 << TXCIE1);
  UCSR1C = (parity << UPM10) |
           ((stop_bits == 2 ? 1 : 0) << USBS1) |
           ((char_size - 5) << UCSZ10);

  g_baud_rate = baud_rate;
  g_char_size = char_size;
  g_parity = parity;
  g_stop_bits = stop_bits;

  return 0;
}

uint8_t serial_available(void) {
  uint8_t result = 0;

  // Disable our receive interrupt.
  UCSR1B &= ~(1 << RXCIE1);
  result = g_recv_buffer_end != g_recv_buffer_start;

  // Now -re-enable our interrupt.
  UCSR1B |= (1 << RXCIE1);
  return result;
}

int16_t serial_getchar(void) {
  int16_t result = 0;

  // Disable our receive interrupt.
  UCSR1B &= ~(1 << RXCIE1);

  if (g_recv_buffer_end == g_recv_buffer_start) {
    result = -1;
  } else {
    result = g_recv_buffer[g_recv_buffer_start];
    g_recv_buffer_start = (g_recv_buffer_start + 1) % BUFFER_SIZE;
  }

  // Re-enable our receive interrupt.
  UCSR1B |= (1 << RXCIE1);

  return result;
}

uint8_t serial_putchar(uint8_t data) {
  uint8_t result;

  // Ensure that the transmit interrupt is off.
  UCSR1B &= ~(1 << UDRIE1);

  // Now look to see if we have room.
  uint8_t next = (g_send_buffer_end + 1) % BUFFER_SIZE;
  if (next == g_send_buffer_start) {
    result = SERIAL_ERR_OVERFLOW;
  } else {
    g_send_buffer[g_send_buffer_end] = data;
    g_send_buffer_end = next;
    result = 0;
  }

  /* If there is some data in the buffer now, then start the interrupt
   * back up. */
  if (g_send_buffer_end != g_send_buffer_start) {
    // If we are half duplex, disable the receiver.
    if (!g_full_duplex) {
      UCSR1B &= ~(1 << RXEN1);
    }
    UCSR1B |= (1 << UDRIE1);
  }

  return result;
}

uint8_t serial_write(const uint8_t* data, uint16_t size) {
  // TODO jpieper: This could be optimized a lot.
  for (uint16_t i = 0; i < size; i++) {
    uint8_t result = serial_putchar(data[i]);
    if (result != 0) { return result; }
  }
  return 0;
}

uint32_t serial_get_baud(void) {
  return g_baud_rate;
}

uint8_t serial_get_parity(void) {
  return g_parity;
}

uint8_t serial_get_char_size(void) {
  return g_char_size;
}

uint8_t serial_get_stop_bits(void) {
  return g_stop_bits;
}

uint8_t serial_get_full_duplex(void) {
  return g_full_duplex;
}

ISR(USART1_RX_vect) {
  uint8_t next = (g_recv_buffer_end + 1) % BUFFER_SIZE;
  uint8_t data = UDR1;
  if (next != g_recv_buffer_start) {
    g_recv_buffer[g_recv_buffer_end] = data;
    g_recv_buffer_end = next;
  } else {
    g_error |= SERIAL_ERR_RX_OVERFLOW;
  }
}

ISR(USART1_UDRE_vect) {
  if (g_send_buffer_end != g_send_buffer_start) {
    UDR1 = g_send_buffer[g_send_buffer_start];
    g_send_buffer_start = (g_send_buffer_start + 1) % BUFFER_SIZE;
  }

  if (g_send_buffer_end == g_send_buffer_start) {
    // Turn off this interrupt because we're all done.
    UCSR1B &= ~(1 << UDRIE1);
  }
}

ISR(USART1_TX_vect) {
  if (g_send_buffer_end == g_send_buffer_start) {
    /* If we have finished transmitting, with nothing left to send,
     * then re-enable the receiver. */
    if (!g_full_duplex) {
      UCSR1B |= (1 << RXEN1);
    }
  }
}
