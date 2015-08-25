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

#include "telemetry_registry.h"

namespace legtool {
void TelemetryRegistry::ObserveSchema(SchemaHandler handler) {
  for (const auto& pair: records_) {
    RecordProperties properties;
    properties.name = pair.first;
    properties.schema = pair.second->schema();
    properties.serialized_data_signal = pair.second->signal();
    handler(properties);
  }
  observers_.push_back(handler);
}

void TelemetryRegistry::RegisterImpl(const std::string& name,
                                     std::unique_ptr<Base>&& record) {
  RecordProperties properties;
  properties.name = name;
  properties.schema = record->schema();
  properties.serialized_data_signal = record->signal();

  records_.insert(std::make_pair(name, std::move(record)));

  for (auto observer: observers_) {
    observer(properties);
  }
}

}
