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

#include "gimbal/telemetry_manager.h"

#include <boost/test/auto_unit_test.hpp>

#include "base/visitor.h"
#include "gimbal/lock_manager.h"
#include "gimbal/test/stream_test.h"

namespace {
struct Struct1 {
  uint32_t val1 = 34;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(val1));
  }
};

struct Struct2 {
  uint8_t val10 = 10;
  uint16_t val11 = 11;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(val10));
    a->Visit(MJ_NVP(val11));
  }
};

bool StartsWith(const std::string& line, const std::string& to_find) {
  return line.substr(0, to_find.size()) == to_find;
}

struct Fixture {
  SizedPool<> pool;
  test::TestWriteStream stream;
  LockManager lock_manager;
  TelemetryManager dut{pool, stream, lock_manager};
  Struct1 struct1;
  Struct2 struct2;
};
}

BOOST_FIXTURE_TEST_CASE(TelemetryManagerTest, Fixture) {
  dut.Register(gsl::ensure_z("struct1"), &struct1);
  dut.Register(gsl::ensure_z("struct2"), &struct2);

  int count = 0;
  int error = 0;
  auto callback = [&](int err) { count++; error = err; };
  dut.Command(gsl::ensure_z("list"), callback);
  BOOST_CHECK_EQUAL(count, 1);
  BOOST_CHECK_EQUAL(error, 0);
  std::string expected = R"XX(struct1
struct2
OK
)XX";
  BOOST_CHECK_EQUAL(stream.ostr.str(), expected);

  count = 0;
  stream.ostr.str("");
  dut.Command(gsl::ensure_z("get struct2"), callback);
  BOOST_CHECK_EQUAL(count, 1);
  BOOST_CHECK_EQUAL(error, 0);
  BOOST_CHECK(StartsWith(stream.ostr.str(), "emit struct2\n"));

  count = 0;
  stream.ostr.str("");
  dut.Command(gsl::ensure_z("schema struct2"), callback);
  BOOST_CHECK_EQUAL(count, 1);
  BOOST_CHECK_EQUAL(error, 0);
  BOOST_CHECK(StartsWith(stream.ostr.str(), "schema struct2\n"));
}

BOOST_FIXTURE_TEST_CASE(TelemetryManagerEmitTest, Fixture) {
  auto updated1 = dut.Register(gsl::ensure_z("struct1"), &struct1);
  auto updated2 = dut.Register(gsl::ensure_z("struct2"), &struct2);

  int count = 0;
  int error = 0;
  auto callback = [&](int err) { count++; error = err; };

  auto Poll = [&]() {
    dut.PollMillisecond();
    for (int i = 0; i < 1000; i++) {
      dut.Poll();
    }
  };

  updated1();
  // Ensure that nothing came out, and that nothing comes out for a
  // large number of milliseconds.
  for (int i = 0; i < 1000; i++) {
    BOOST_CHECK_EQUAL(stream.ostr.str(), "");
    Poll();
  }

  // Now set a rate that should only result in stuff coming out when
  // we update.
  dut.Command(gsl::ensure_z("rate struct1 1"), callback);
  BOOST_CHECK_EQUAL(count, 1);
  BOOST_CHECK_EQUAL(error, 0);
  BOOST_CHECK_EQUAL(stream.ostr.str(), "OK\n");
  stream.ostr.str("");

  // Nothing should come out spontaneously still.
  for (int i = 0; i < 1000; i++) {
    BOOST_CHECK_EQUAL(stream.ostr.str(), "");
    Poll();
  }

  // But as soon as we update it, we should see something after a
  // Poll.
  updated1();
  BOOST_CHECK_EQUAL(stream.ostr.str(), "");
  dut.Poll();
  BOOST_CHECK(StartsWith(stream.ostr.str(), "emit struct1\n"));
  stream.ostr.str("");

  // Now set a time rate.
  count = 0;
  dut.Command(gsl::ensure_z("rate struct1 20"), callback);
  BOOST_CHECK_EQUAL(count, 1);
  BOOST_CHECK_EQUAL(error, 0);
  BOOST_CHECK_EQUAL(stream.ostr.str(), "OK\n");
  stream.ostr.str("");

  // Now, calling update followed by poll should have no effect.
  updated1();
  dut.Poll();
  BOOST_CHECK_EQUAL(stream.ostr.str(), "");

  // But waiting a while should.
  for (int i = 0; i < 20; i++) {
    BOOST_CHECK_EQUAL(stream.ostr.str(), "");
    Poll();
  }

  BOOST_CHECK(StartsWith(stream.ostr.str(), "emit struct1\n"));

  // And it should come out again in the same number of cycles.
  stream.ostr.str("");

  for (int i = 0; i < 20; i++) {
    BOOST_CHECK_EQUAL(stream.ostr.str(), "");
    Poll();
  }

  BOOST_CHECK(StartsWith(stream.ostr.str(), "emit struct1\n"));
}
