// Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <stdlib.h>

#include <cstring>

#include "gpio.h"
#include "i2c.h"
#include "iwdg.h"

#include "uart_stream.h"
#include "usb_cdc_stream.h"

void *operator new(size_t size) throw() { return malloc(size); }
void operator delete(void* p) throw() { free(p); }

namespace {
int g_rx_count = 0;
int g_usb_rx_count = 0;
char g_usb_buffer[2] = {};
char g_uart_buffer[2] = {};
}

extern "C" {

void Read(AsyncReadStream& stream, gsl::string_span buffer, int* count) {
  stream.AsyncReadSome(buffer,
                       [&stream, count, buffer](int, int) {
                         (*count)++;
                         Read(stream, buffer, count);
                       });
}

void cpp_gimbal_main() {
  UsbCdcStream usb_cdc;
  UartStream uart2(&huart2);

  // For now, lets try to flash an LED with no sleeps or anything.
  uint32_t old_tick = 0;
  int cycle = 0;
  char buffer[64] = {};

  bool usb_write = false;
  int usb_tx_count = 0;

  bool uart_write = false;
  int uart_tx_count = 0;

  Read(usb_cdc, gsl::string_span(g_usb_buffer), &g_usb_rx_count);
  Read(uart2, gsl::string_span(g_uart_buffer), &g_rx_count);

  while (1) {
    HAL_IWDG_Refresh(&hiwdg);
    uint32_t new_tick = HAL_GetTick();
    if (new_tick != old_tick) {
      usb_cdc.PollMillisecond();
    }
    if (new_tick != old_tick && (new_tick % 250) == 0) {
      uint8_t i2c_buf[4] = {};
      auto i2c_status = HAL_I2C_Mem_Read(
          &hi2c1, 0x32, 0x20, I2C_MEMADD_SIZE_8BIT,
          i2c_buf, sizeof(i2c_buf), 100);

      snprintf(buffer, sizeof(buffer) - 1, "tick: %lu cpt: %d rx:%d ur:%d ut:%d i2cs:%d i2cd:%d\r\n",
               new_tick, uart_tx_count, g_rx_count, g_usb_rx_count, usb_tx_count,
               static_cast<int>(i2c_status), static_cast<int>(i2c_buf[0]));


      if (!uart_write) {
        uart_write = true;
        AsyncWrite(uart2, gsl::ensure_z(buffer),
                   [&](int) {
                     uart_write = false;
                     uart_tx_count++;
                   });
      }
      if (!usb_write) {
        usb_write = true;
        AsyncWrite(usb_cdc, gsl::ensure_z(buffer),
                   [&](int){
                     usb_write = false;
                     usb_tx_count++;
                   });
      }

      cycle = (cycle + 1) % 8;
      if (cycle & 1) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
      }
      if (cycle & 2) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
      }
      if (cycle & 4) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
      }
    }
    old_tick = new_tick;
  }
}
}
