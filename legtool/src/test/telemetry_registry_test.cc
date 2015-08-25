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

#include <boost/test/auto_unit_test.hpp>

#include "visitor.h"

using namespace legtool;

namespace {
struct TestData {
  int8_t foo = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(LT_NVP(foo));
  }
};
}

BOOST_AUTO_TEST_CASE(TelemetryBasicTest) {
  TelemetryRegistry registry;
  auto callable = registry.Register<TestData>("data1");
  TestData data;

  // Nothing much should happen.
  callable(&data);

  std::vector<TelemetryRegistry::RecordProperties> properties;

  // Now we can register a schema observer.
  registry.ObserveSchema(
      [&](const TelemetryRegistry::RecordProperties& p) {
        properties.push_back(p);
      });

  // And we should get the one that we've already registered.
  BOOST_REQUIRE_EQUAL(properties.size(), 1);
  BOOST_CHECK_EQUAL(properties[0].name, "data1");
  BOOST_CHECK(properties[0].serialized_data_signal != nullptr);

  std::string bin;
  properties[0].serialized_data_signal->connect(
      [&](const std::string& d) { bin = d; });
  BOOST_CHECK_EQUAL(bin, "");

  callable(&data);
  BOOST_CHECK(!bin.empty());

  // And if we register something new, the schema observer should see
  // it.
  registry.Register<TestData>("data2");

  BOOST_REQUIRE_EQUAL(properties.size(), 2);
  BOOST_CHECK_EQUAL(properties[1].name, "data2");

  // We can get a hold of a correctly typed signal when we want one.
  auto signal = registry.GetConcreteSignal<TestData>("data1");
  BOOST_CHECK(signal != nullptr);
  data.foo = 30;

  TestData input;
  signal->connect([&](const TestData* value) { input = *value; });

  BOOST_CHECK_EQUAL(input.foo, 0);
  callable(&data);
  BOOST_CHECK_EQUAL(input.foo, 30);
}
