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

#include "gimbal/persistent_config.h"

#include <boost/test/auto_unit_test.hpp>

#include "base/visitor.h"

#include "gimbal/test/flash_test.h"
#include "gimbal/test/stream_test.h"

namespace {
struct TestStruct {
  int vi = 0;
  float vf = 1.0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(vi));
    a->Visit(MJ_NVP(vf));
  }
};

struct BigStruct {
  uint8_t u8 = 3;
  TestStruct sub;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(u8));
    a->Visit(MJ_NVP(sub));
  }
};
}

BOOST_AUTO_TEST_CASE(SetArchiveTest) {
  BigStruct bs;

  BOOST_CHECK_EQUAL(bs.u8, 3);
  detail::SetArchive(gsl::ensure_z("u8"), gsl::ensure_z("9")).Accept(&bs);
  BOOST_CHECK_EQUAL(bs.u8, 9);

  BOOST_CHECK_EQUAL(bs.sub.vi, 0);
  detail::SetArchive(gsl::ensure_z("sub.vi"),
                     gsl::ensure_z("0x10")).Accept(&bs);
  BOOST_CHECK_EQUAL(bs.sub.vi, 16);

  BOOST_CHECK_EQUAL(bs.sub.vf, 1.0);
  detail::SetArchive(gsl::ensure_z("sub.vf"),
                     gsl::ensure_z("2.5")).Accept(&bs);
  BOOST_CHECK_EQUAL(bs.sub.vf, 2.5);
}

BOOST_AUTO_TEST_CASE(ReadArchiveTest) {
  BigStruct bs;

  char buffer[256] = {};
  gsl::string_span span(buffer);

  bs.sub.vi = 12345;
  bs.sub.vf = 8.25;

  bool done = false;
  test::TestWriteStream test_stream;
  detail::ReadArchive("sub.vi", span, test_stream, [&done](ErrorCode error) {
      Expects(!error);
      done = true;
    }).Accept(&bs);

  BOOST_CHECK_EQUAL(done, true);
  BOOST_CHECK_EQUAL(test_stream.ostr.str(), "12345");

  test_stream.ostr.str("");

  done = false;
  detail::ReadArchive("sub.vf", span, test_stream, [&done](ErrorCode error) {
      Expects(!error);
      done = true;
    }).Accept(&bs);
  BOOST_CHECK_EQUAL(done, true);
  BOOST_CHECK_EQUAL(test_stream.ostr.str(), "8.250000");
}

BOOST_AUTO_TEST_CASE(EnumerateArchiveTest) {
  BigStruct bs;

  test::TestWriteStream test_stream;
  detail::EnumerateArchive::Context context;
  context.root_prefix = gsl::ensure_z("bigstruct");
  char buffer[256] = {};
  context.buffer = buffer;
  context.stream = &test_stream;

  bool done = false;
  int error = 0;
  int count = 0;
  context.callback = [&](ErrorCode ec) { done = true; error = ec; count++; };
  context.evaluate_enumerate_archive = [&]() {
    uint16_t current_index = 0;
    bool done = false;
    detail::EnumerateArchive archive(&context, context.root_prefix,
                                     &current_index, &done, nullptr);
    archive.Accept(&bs);
    return done;
  };

  context.evaluate_enumerate_archive();

  BOOST_CHECK_EQUAL(done, true);
  BOOST_CHECK_EQUAL(error, 0);
  BOOST_CHECK_EQUAL(count, 1);
  std::string expected = R"XX(bigstruct.u8 3
bigstruct.sub.vi 0
bigstruct.sub.vf 1.000000
)XX";
  BOOST_CHECK_EQUAL(test_stream.ostr.str(), expected);
}

namespace {
struct OtherStruct {
  int vint = 0;
  uint16_t v2 = 1;
  uint32_t v3 = 2;
  float vfloat = 3.5;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(vint));
    a->Visit(MJ_NVP(v2));
    a->Visit(MJ_NVP(v3));
    a->Visit(MJ_NVP(vfloat));
  }
};

struct PersistentConfigFixture {
  SizedPool<> pool;
  test::FlashTest flash;
  test::TestWriteStream test_stream;
  PersistentConfig config{pool, flash, test_stream};
  BigStruct bs;
  OtherStruct os;

  PersistentConfigFixture() {
    config.Register(gsl::ensure_z("bigs"), &bs);
    config.Register(gsl::ensure_z("other"), &os);
  }
};
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigTest, PersistentConfigFixture) {
  int count = 0;
  int error = 0;
  auto callback = [&](int err) { count++; error = err; };

  config.Command(gsl::ensure_z("bogus"), callback);

  BOOST_CHECK_EQUAL(count, 1);
  BOOST_CHECK_EQUAL(error, 0);
  BOOST_CHECK_EQUAL(test_stream.ostr.str(), "unknown command\n");

  count = 0;
  test_stream.ostr.str("");
  BOOST_REQUIRE_EQUAL(os.vfloat, 3.5);
  config.Command(gsl::ensure_z("set other.vfloat 4.5"), callback);

  BOOST_CHECK_EQUAL(count, 1);
  BOOST_CHECK_EQUAL(error, 0);
  BOOST_CHECK_EQUAL(test_stream.ostr.str(), "OK\n");
  BOOST_CHECK_EQUAL(os.vfloat, 4.5);

  count = 0;
  test_stream.ostr.str("");
  config.Command(gsl::ensure_z("get bigs.u8"), callback);

  BOOST_CHECK_EQUAL(count, 1);
  BOOST_CHECK_EQUAL(error, 0);
  BOOST_CHECK_EQUAL(test_stream.ostr.str(), "3\n");

  count = 0;
  test_stream.ostr.str("");
  config.Command(gsl::ensure_z("enumerate"), callback);

  BOOST_CHECK_EQUAL(count, 1);
  BOOST_CHECK_EQUAL(error, 0);
  std::string expected = R"XX(bigs.u8 3
bigs.sub.vi 0
bigs.sub.vf 1.000000
other.vint 0
other.v2 1
other.v3 2
other.vfloat 4.500000
OK
)XX";
  BOOST_CHECK_EQUAL(test_stream.ostr.str(), expected);
}

BOOST_FIXTURE_TEST_CASE(PersistentConfigFlashTest, PersistentConfigFixture) {
  int count = 0;
  int error = 0;
  auto callback = [&](int err) { count++; error = err; };

  config.Command("write", callback);

  BOOST_CHECK_EQUAL(count, 1);
  BOOST_CHECK_EQUAL(error, 0);
  BOOST_CHECK_EQUAL(test_stream.ostr.str(), "OK\n");

  SimpleIStream raw_stream(flash.buffer_, sizeof(flash.buffer_));
  mjmech::base::TelemetryReadStream<SimpleIStream> stream(raw_stream);

  BOOST_CHECK_EQUAL(stream.ReadString(), "bigs");
  BOOST_CHECK_EQUAL(stream.Read<uint32_t>(), 0x617e3c8d); // CRC
  std::string bigs_data = stream.ReadString();
  BOOST_CHECK_EQUAL(bigs_data.size(), static_cast<std::size_t>(9));

  BOOST_CHECK_EQUAL(stream.ReadString(), "other");
  BOOST_CHECK_EQUAL(stream.Read<uint32_t>(), 0x1cac97a0); // CRC
  std::string other_data = stream.ReadString();
  BOOST_CHECK_EQUAL(other_data.size(), static_cast<std::size_t>(14));

  BOOST_CHECK_EQUAL(stream.ReadString(), "");

  bs.u8 = 9;
  os.v2 = 10;

  count = 0;
  test_stream.ostr.str("");

  config.Command("load", callback);

  BOOST_CHECK_EQUAL(count, 1);
  BOOST_CHECK_EQUAL(error, 0);
  BOOST_CHECK_EQUAL(test_stream.ostr.str(), "OK\n");

  BOOST_CHECK_EQUAL(bs.u8, 3);
  BOOST_CHECK_EQUAL(os.v2, 1);
}
