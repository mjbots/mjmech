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

#include "as5048_driver.h"
#include "bldc_encoder.h"
#include "bmi160_driver.h"
#include "command_manager.h"
#include "fire_control.h"
#include "gimbal_herkulex_operations.h"
#include "gimbal_stabilizer.h"
#include "herkulex_protocol.h"
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
#include "stm32_pwm.h"
#include "stm32_raw_i2c.h"
#include "stm32_timex_complement_pwm.h"
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
  bool herkulex_init = false;
  int32_t herkulex_error = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(command_manager_init));
    a->Visit(MJ_NVP(bmi160_init));
    a->Visit(MJ_NVP(bmi160_error));
    a->Visit(MJ_NVP(herkulex_init));
    a->Visit(MJ_NVP(herkulex_error));
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
  UartStream uart(&huart6, GPIOC, GPIO_PIN_6);

  auto& debug_stream = usb_cdc;
  auto& herkulex_stream = uart;

  Stm32Clock clock;

  SizedPool<> pool;
  Stm32RawI2C::Parameters parameters;
  parameters.speed = 400000;
  Stm32GpioPin i2c1_sda(GPIOB, GPIO_PIN_7);
  Stm32GpioPin i2c1_scl(GPIOB, GPIO_PIN_6);
  Stm32RawI2C i2c1(pool, 1, i2c1_scl, i2c1_sda, parameters, clock);

  Stm32GpioPin i2c2_sda(GPIOB, GPIO_PIN_9);
  Stm32GpioPin i2c2_scl(GPIOB, GPIO_PIN_10);
  Stm32RawI2C i2c2(pool, 2, i2c2_scl, i2c2_sda, parameters, clock);
  Stm32HalSPI spi1(pool, 3, GPIOC, GPIO_PIN_15);
  Stm32Flash flash;
  PersistentConfig config(pool, flash);
  LockManager lock_manager;
  TelemetryManager telemetry(pool, lock_manager);
  CommandManager command_manager(pool, debug_stream, lock_manager);
  SystemInfo system_info(pool, telemetry, clock);
  Stm32AnalogSampler analog_sampler(pool, clock, config, telemetry);
  Bmi160Driver bmi160(pool, gsl::ensure_z("pimu"),
                      i2c1, clock, config, telemetry);
  As5048Driver yaw_encoder(pool, gsl::ensure_z("yawenc"),
                           nullptr, &spi1, clock, config, telemetry);
  BldcEncoder yaw_bldc_encoder(pool, gsl::ensure_z("yawblenc"),
                               yaw_encoder, clock, config, telemetry);
  As5048Driver pitch_encoder(pool, gsl::ensure_z("pitchenc"),
                             &i2c2, nullptr, clock, config, telemetry);
  BldcEncoder pitch_bldc_encoder(pool, gsl::ensure_z("pitchblenc"),
                                 pitch_encoder, clock, config, telemetry);
  Stm32BldcPwm motor1(&htim2, TIM_CHANNEL_1,
                      &htim2, TIM_CHANNEL_2,
                      &htim2, TIM_CHANNEL_3);
  Stm32BldcPwm motor2(&htim3, TIM_CHANNEL_1,
                      &htim3, TIM_CHANNEL_2,
                      &htim4, TIM_CHANNEL_3);
  MahonyImu imu(pool, clock, config, telemetry, *bmi160.data_signal());

  Stm32GpioPin bldc_sleep(GPIOA, GPIO_PIN_6, true);
  Stm32GpioPin bldc_reset(GPIOA, GPIO_PIN_7, true);
  bldc_sleep.Set(false);
  bldc_reset.Set(false);

  Stm32GpioPin boost_enable(GPIOC, GPIO_PIN_3);
  Stm32GpioPin motor_enable(GPIOC, GPIO_PIN_13);
  Stm32GpioPin torque_led(GPIOB, GPIO_PIN_14, true);
  GimbalStabilizer stabilizer(pool, clock, config, telemetry,
                              *imu.data_signal(),
                              boost_enable, motor_enable, motor1, motor2,
                              *yaw_bldc_encoder.data_signal(),
                              *pitch_bldc_encoder.data_signal(),
                              torque_led);

  Stm32GpioPin laser_enable(GPIOA, GPIO_PIN_10);
  Stm32GpioPin pwm_enable(GPIOC, GPIO_PIN_14, true);
  Stm32Pwm aeg_pwm(&htim3, TIM_CHANNEL_3);
  Stm32Pwm agitator_pwm(&htim3, TIM_CHANNEL_4);
  Stm32GpioPin arm_switch(GPIOB, GPIO_PIN_12);
  Stm32GpioPin arm_led(GPIOB, GPIO_PIN_13, true);
  FireControl fire_control(pool, clock, config, telemetry,
                           laser_enable, pwm_enable, aeg_pwm, agitator_pwm,
                           arm_switch, arm_led);

  GimbalHerkulexOperations operations(
      stabilizer, imu, yaw_bldc_encoder, fire_control);
  HerkulexProtocol herkulex(pool, herkulex_stream, operations);

  command_manager.RegisterHandler(gsl::ensure_z("conf"), config);
  command_manager.RegisterHandler(gsl::ensure_z("tel"), telemetry);
  command_manager.RegisterHandler(gsl::ensure_z("gim"), stabilizer);
  command_manager.RegisterHandler(gsl::ensure_z("fire"), fire_control);

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

  herkulex.AsyncStart([&](int error) {
      if (!error) {
        system_status.herkulex_init = true;
        system_status.herkulex_error = error;
      }
    });

  uint32_t old_tick = 0;
  while (1) {
    uint32_t new_tick = HAL_GetTick();
    if (new_tick != old_tick) {
      usb_cdc.PollMillisecond();
      telemetry.PollMillisecond();
      system_info.PollMillisecond();
      analog_sampler.PollMillisecond();
      yaw_bldc_encoder.PollMillisecond();
      pitch_bldc_encoder.PollMillisecond();
      stabilizer.PollMillisecond();
      fire_control.PollMillisecond();
      system_status.timestamp = clock.timestamp();
      HAL_IWDG_Refresh(&hiwdg);

      UpdateLEDs(new_tick);
    }
    old_tick = new_tick;

    usb_cdc.Poll();
    uart.Poll();
    i2c1.Poll();
    i2c2.Poll();
    spi1.Poll();
    telemetry.Poll();
    command_manager.Poll();
    bmi160.Poll();
    yaw_encoder.Poll();
    pitch_encoder.Poll();
    system_info.MainLoopCount();
  }
}
}
