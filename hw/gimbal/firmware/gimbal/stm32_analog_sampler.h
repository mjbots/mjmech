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

#pragma once

#include "base/visitor.h"

#include "pool_ptr.h"

class Clock;
class PersistentConfig;
class TelemetryManager;

class Stm32AnalogSampler {
 public:
  Stm32AnalogSampler(Pool&, Clock&, PersistentConfig&, TelemetryManager&);
  ~Stm32AnalogSampler();

  void PollMillisecond();

  struct Data {
    uint32_t timestamp = 0;

    uint16_t raw_vrefint = 0;
    uint16_t raw_vbat = 0;
    uint16_t raw_8v = 0;
    uint16_t raw_12v = 0;
    uint16_t raw_temperature = 0;

    float power_vbat = 0.0;
    float power_8v = 0.0;
    float power_12v = 0.0;
    float temperature_C = 0.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));

      a->Visit(MJ_NVP(raw_vrefint));
      a->Visit(MJ_NVP(raw_vbat));
      a->Visit(MJ_NVP(raw_8v));
      a->Visit(MJ_NVP(raw_12v));
      a->Visit(MJ_NVP(raw_temperature));

      a->Visit(MJ_NVP(power_vbat));
      a->Visit(MJ_NVP(power_8v));
      a->Visit(MJ_NVP(power_12v));
      a->Visit(MJ_NVP(temperature_C));
    }
  };

  const Data* data() const;

 private:
  class Impl;
  PoolPtr<Impl> impl_;
};
