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

#include "stm32_analog_sampler.h"

#include "adc.h"

#include "clock.h"
#include "persistent_config.h"
#include "telemetry_manager.h"

namespace {
// The resistor divider for the external voltages is 10k by 100k, or a
// factor of 11.
int kExternalDivider = 11;
}

class Stm32AnalogSampler::Impl {
 public:
  Impl(Clock& clock,
       PersistentConfig& persistent_config,
       TelemetryManager& telemetry)
      : clock_(clock) {
    data_updater_ = telemetry.Register(gsl::ensure_z("power"), &data_);
  }

  void PollMillisecond() {
    if (configuring_) {
      HAL_ADC_Start(&hadc1);
      configuring_ = false;
      return;
    }
    if (state_ != kIdle) {
      // We are looking for a conversion to have completed.
      if (!__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC)) { return; }

      const uint16_t value = HAL_ADC_GetValue(&hadc1);
      switch (state_) {
        case kIdle: { assert(false); break; }
        case kVRefInt: { data_.raw_vrefint = value; break; }
        case kVBat: { data_.raw_vbat = value; break; }
        case k8V: { data_.raw_8v = value; break; }
        case k12V: { data_.raw_12v = value; break; }
        case kTemperature: { data_.raw_temperature = value; break; }
      }
    }

    if (state_ == kFinal) {
      // Emit our result.
      data_.timestamp = clock_.timestamp();
      const float vadc = 1.2f / (data_.raw_vrefint / 4096.0f);
      data_.power_vbat = data_.raw_vbat / 4096.0f * 4 * vadc;
      data_.power_8v = data_.raw_8v / 4096.0f * kExternalDivider * vadc;
      data_.power_12v = data_.raw_12v / 4096.0f * kExternalDivider * vadc;

      const float vtemp = data_.raw_temperature / 4096.0 * vadc;
      data_.temperature_C = (vtemp - 0.75) / 0.00275 + 25;

      data_updater_();
    }

    // Finally, advance the state and start our next conversion.
    const int next_state = std::max(1, (state_ + 1) % (kFinal + 1));
    state_ = static_cast<State>(next_state);

    __attribute__((unused)) ADC_ChannelConfTypeDef adc_conf;
    adc_conf.Rank = 1;
    adc_conf.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    adc_conf.Channel = [&]() -> uint32_t {
      switch (state_) {
        case kIdle: { assert(false); return 0; }
        case kVRefInt: { return ADC_CHANNEL_VREFINT; }
        case kVBat: { return ADC_CHANNEL_VBAT; }
        case k8V: { return ADC_CHANNEL_11; }
        case k12V: { return ADC_CHANNEL_12; }
        case kTemperature: { return ADC_CHANNEL_TEMPSENSOR; }
      }
      assert(false);
      return 0;
    }();

    // The stupid HAL layer sets, but never clears these flags, so we
    // can't switch between things without clearing them ourselves.
    ADC->CCR &= ~ADC_CCR_TSVREFE;
    ADC->CCR &= ~ADC_CCR_VBATE;
    HAL_ADC_ConfigChannel(&hadc1, &adc_conf);
    configuring_ = true;
  }

  Clock& clock_;
  Data data_;
  StaticFunction<void ()> data_updater_;

  enum State {
    kIdle,
    kVRefInt,
    kVBat,
    k8V,
    k12V,
    kTemperature,
    kFinal = kTemperature,
  };

  State state_ = kIdle;
  bool configuring_ = false;
};

Stm32AnalogSampler::Stm32AnalogSampler(Pool& pool,
                                       Clock& clock,
                                       PersistentConfig& persistent_config,
                                       TelemetryManager& telemetry)
: impl_(&pool, clock, persistent_config, telemetry) {
}

Stm32AnalogSampler::~Stm32AnalogSampler() {}

void Stm32AnalogSampler::PollMillisecond() {
  impl_->PollMillisecond();
}
