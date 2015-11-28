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
#include "iwdg.h"
#include "usart.h"
#include "usb_cdc_stream.h"

void *operator new(size_t size) throw() { return malloc(size); }
void operator delete(void* p) throw() { free(p); }

namespace {
volatile int g_complete = 0;
volatile bool g_receiving = false;
volatile int g_rx_count = 0;
volatile int g_usb_rx_count = 0;
char g_usb_buffer[2] = {};
}

extern "C" {

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  g_complete++;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  g_receiving = false;
  g_rx_count += 1;
}

void Read(AsyncReadStream& stream) {
  stream.AsyncReadSome(gsl::string_span(g_usb_buffer),
                       [&](int, int) {
                         g_usb_rx_count++;
                         Read(stream);
                       });
}

void cpp_gimbal_main() {
  UsbCdcStream usb_cdc;

  // For now, lets try to flash an LED with no sleeps or anything.
  uint32_t old_tick = 0;
  int cycle = 0;
  char buffer[64] = {};
  char rx_buffer[2] = {};
  bool usb_write = false;
  int usb_tx_count = 0;
  Read(usb_cdc);
  while (1) {
    HAL_IWDG_Refresh(&hiwdg);
    uint32_t new_tick = HAL_GetTick();
    if (new_tick != old_tick) {
      usb_cdc.PollMillisecond();
    }
    if (new_tick != old_tick && (new_tick % 250) == 0) {
      snprintf(buffer, sizeof(buffer) - 1, "tick: %lu cpt: %d rx:%d ur:%d ut:%d\r\n",
               new_tick, g_complete, g_rx_count, g_usb_rx_count, usb_tx_count);
      HAL_UART_Transmit_DMA(
          &huart2, reinterpret_cast<uint8_t*>(buffer),
          std::strlen(buffer));
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

    if (!g_receiving) {
      g_receiving = true;
      HAL_UART_Receive_DMA(
          &huart2,
          reinterpret_cast<uint8_t*>(rx_buffer),
          1);
    }
  }
}
}
