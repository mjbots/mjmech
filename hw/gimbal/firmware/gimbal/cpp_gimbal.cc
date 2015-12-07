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

#include "base/visitor.h"

#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"

#include "bmi160_driver.h"
#include "command_manager.h"
#include "lock_manager.h"
#include "persistent_config.h"
#include "pool_ptr.h"
#include "stm32_clock.h"
#include "stm32_flash.h"
#include "stm32_hal_i2c.h"
#include "stm32_hal_spi.h"
#include "stm32_raw_i2c.h"
#include "system_info.h"
#include "telemetry_manager.h"
#include "uart_stream.h"
#include "usb_cdc_stream.h"

void *operator new(size_t size) throw() { return malloc(size); }
void operator delete(void* p) throw() { free(p); }

namespace {
struct SystemStatus {
  uint32_t timestamp = 0;
  bool command_manager_init = false;
  bool bmi160_init = false;
  int32_t bmi160_error = 0;
  uint32_t adc_value = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(command_manager_init));
    a->Visit(MJ_NVP(bmi160_init));
    a->Visit(MJ_NVP(bmi160_error));
    a->Visit(MJ_NVP(adc_value));
  }
};

void UpdateLEDs(uint32_t count) {
  int cycle = count / 250;
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
}

extern "C" {

void cpp_gimbal_main() {
  UsbCdcStream usb_cdc;
  UartStream uart2(&huart2);

  auto& debug_stream = uart2;
  auto& time_stream = usb_cdc;

  SizedPool<> pool;
  Stm32RawI2C i2c1(pool, 1, Stm32RawI2C::Parameters());
  Stm32HalSPI spi1(pool, 1, GPIOE, GPIO_PIN_3);
  Stm32Flash flash;
  PersistentConfig config(pool, flash, debug_stream);
  LockManager lock_manager;
  TelemetryManager telemetry(pool, debug_stream, lock_manager);
  CommandManager command_manager(pool, debug_stream, lock_manager);
  Stm32Clock clock;
  SystemInfo system_info(pool, telemetry, clock);
  Bmi160Driver bmi160(pool, gsl::ensure_z("pimu"),
                      i2c1, clock, config, telemetry);

  command_manager.Register(
      gsl::ensure_z("conf"),
      [&](const gsl::cstring_span& args, ErrorCallback cbk) {
        config.Command(args, cbk);
      });
  command_manager.Register(
      gsl::ensure_z("tel"),
      [&](const gsl::cstring_span& args, ErrorCallback cbk) {
        telemetry.Command(args, cbk);
      });

  SystemStatus system_status;
  telemetry.Register(gsl::ensure_z("system_status"), &system_status);

  command_manager.AsyncStart([&](int error) {
      if (!error) {
        system_status.command_manager_init = true;
      }
    });

  bmi160.AsyncStart([&](int error) {
      if (!error) {
        system_status.bmi160_init = true;
        system_status.bmi160_error = error;
      }
    });

  char buffer[100] = {};
  bool uart_write = false;

  bool spi_write = false;
  int spi_count = 0;
  int spi_error = 0;
  char spi_write_buf[2] = { 0xcf, 0x00 };
  char spi_read_buf[2] = {};

  uint32_t old_tick = 0;
  while (1) {
    uint32_t new_tick = HAL_GetTick();
    if (new_tick != old_tick) {
      usb_cdc.PollMillisecond();
      telemetry.PollMillisecond();
      system_info.PollMillisecond();
      system_status.timestamp = clock.timestamp();

      if ((new_tick % 1000) == 0) {
        snprintf(buffer, sizeof(buffer) - 1, "%lu: spic=%d spie=%d spid=%02X\r\n",
                 clock.timestamp(),
                 spi_count,
                 spi_error,
                 static_cast<int>(static_cast<uint8_t>(spi_read_buf[1])));
        if (!uart_write) {
          uart_write = true;
          AsyncWrite(time_stream, gsl::ensure_z(buffer),
                     [&](int error){ uart_write = false; });
        }

        if (!spi_write) {
          spi_write = true;
          std::memset(spi_read_buf, 0, sizeof(spi_read_buf));
          spi1.AsyncTransaction(spi_write_buf, spi_read_buf, [&](int error) {
              spi_write = false;
              spi_error = error;
              spi_count++;
            });
        }

        HAL_ADC_Start(&hadc1);
      }

      if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC)) {
        system_status.adc_value = HAL_ADC_GetValue(&hadc1);
      }

      UpdateLEDs(new_tick);
    }
    old_tick = new_tick;

    HAL_IWDG_Refresh(&hiwdg);
    usb_cdc.Poll();
    uart2.Poll();
    i2c1.Poll();
    spi1.Poll();
    telemetry.Poll();
    command_manager.Poll();
    bmi160.Poll();
    system_info.MainLoopCount();
  }
}
}
