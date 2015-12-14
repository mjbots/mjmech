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


#include "gpio.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"

#include "stm32f4xx_hal_tim_ex.h"

#include "bmi160_driver.h"
#include "command_manager.h"
#include "gimbal_stabilizer.h"
#include "lock_manager.h"
#include "mahony_imu.h"
#include "persistent_config.h"
#include "pool_ptr.h"
#include "stm32_analog_sampler.h"
#include "stm32_bldc_pwm.h"
#include "stm32_clock.h"
#include "stm32_flash.h"
#include "stm32_gpio_pin.h"
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

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(command_manager_init));
    a->Visit(MJ_NVP(bmi160_init));
    a->Visit(MJ_NVP(bmi160_error));
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
  UartStream uart2(&huart2, GPIOA, GPIO_PIN_2);

  auto& debug_stream = usb_cdc;
  auto& time_stream = uart2;

  Stm32Clock clock;

  SizedPool<> pool;
  Stm32RawI2C i2c1(pool, 1, Stm32RawI2C::Parameters(), clock);
  Stm32HalSPI spi1(pool, 1, GPIOE, GPIO_PIN_3);
  Stm32Flash flash;
  PersistentConfig config(pool, flash);
  LockManager lock_manager;
  TelemetryManager telemetry(pool, lock_manager);
  CommandManager command_manager(pool, debug_stream, lock_manager);
  SystemInfo system_info(pool, telemetry, clock);
  Stm32AnalogSampler analog_sampler(pool, clock, config, telemetry);
  Bmi160Driver bmi160(pool, gsl::ensure_z("pimu"),
                      i2c1, clock, config, telemetry);
  Stm32BldcPwm motor1(&htim3, TIM_CHANNEL_1,
                      &htim3, TIM_CHANNEL_2,
                      &htim3, TIM_CHANNEL_3);
  Stm32BldcPwm motor2(&htim2, TIM_CHANNEL_1,
                      &htim2, TIM_CHANNEL_2,
                      &htim3, TIM_CHANNEL_4);
  MahonyImu imu(pool, clock, config, telemetry, *bmi160.data_signal());

  Stm32GpioPin motor_enable(GPIOA, GPIO_PIN_6);
  GimbalStabilizer stabilizer(pool, clock, config, telemetry,
                              *imu.data_signal(),
                              motor_enable, motor1, motor2);

  command_manager.Register(
      gsl::ensure_z("conf"),
      [&](const gsl::cstring_span& args,
          const CommandManager::Response& response) {
        config.Command(args, response);
      });
  command_manager.Register(
      gsl::ensure_z("tel"),
      [&](const gsl::cstring_span& args,
          const CommandManager::Response& response) {
        telemetry.Command(args, response);
      });
  command_manager.Register(
      gsl::ensure_z("gim"),
      [&](const gsl::cstring_span& args,
          const CommandManager::Response& response) {
        stabilizer.Command(args, response);
      });

  SystemStatus system_status;
  telemetry.Register(gsl::ensure_z("system_status"), &system_status);

  config.Load();

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

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

  uint32_t old_tick = 0;
  while (1) {
    uint32_t new_tick = HAL_GetTick();
    if (new_tick != old_tick) {
      usb_cdc.PollMillisecond();
      telemetry.PollMillisecond();
      system_info.PollMillisecond();
      analog_sampler.PollMillisecond();
      stabilizer.PollMillisecond();
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

        TIM1->CCR2 = (new_tick / 10 + 100) % 2048;
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
