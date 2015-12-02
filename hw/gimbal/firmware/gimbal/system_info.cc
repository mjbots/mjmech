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

#include "system_info.h"

#include "base/visitor.h"

#include "clock.h"
#include "static_function.h"
#include "telemetry_manager.h"

namespace {
struct SystemInfoData {
  uint32_t timestamp = 0;
  uint32_t main_loops_per_10ms = 0;
  uint32_t pool_size = 0;
  uint32_t pool_available = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(main_loops_per_10ms));
    a->Visit(MJ_NVP(pool_size));
    a->Visit(MJ_NVP(pool_available));
  }
};
}

class SystemInfo::Impl {
 public:
  Impl(Pool& pool, TelemetryManager& telemetry, Clock& clock)
      : pool_(pool),
        clock_(clock) {
    data_updater_ = telemetry.Register(gsl::ensure_z("system_info"), &data_);
  }

  void PollMillsecond() {
    ms_count_++;
    if (ms_count_ >= 10) {
      ms_count_ = 0;
    } else {
      return;
    }

    data_.main_loops_per_10ms = main_loops_;
    main_loops_ = 0;

    data_.timestamp = clock_.timestamp();
    data_.pool_size = pool_.size();
    data_.pool_available = pool_.available();

    data_updater_();
  }

  Pool& pool_;
  Clock& clock_;

  uint8_t ms_count_ = 0;
  uint32_t main_loops_ = 0;
  SystemInfoData data_;
  StaticFunction<void ()> data_updater_;
};

SystemInfo::SystemInfo(Pool& pool, TelemetryManager& telemetry, Clock& clock)
    : impl_(&pool, pool, telemetry, clock) {}

SystemInfo::~SystemInfo() {}

void SystemInfo::MainLoopCount() {
  impl_->main_loops_++;
}

void SystemInfo::PollMillisecond() {
  impl_->PollMillsecond();
}
