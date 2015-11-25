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

namespace {
class TestWriteStream : public AsyncWriteStream {
 public:
  ~TestWriteStream() override {}

  void AsyncWriteSome(const gsl::cstring_span& data,
                      SizeCallback callback) override final {
    // We'll only accept one character at a time to ensure that
    // retrying is working.
    ostr.write(&*data.begin(), 1);
    callback(0, 1);
  }

  std::ostringstream ostr;
};
}

BOOST_AUTO_TEST_CASE(ReadArchiveTest) {
  BigStruct bs;

  char buffer[256] = {};
  gsl::string_span span(buffer);

  bs.sub.vi = 12345;
  bs.sub.vf = 8.25;

  bool done = false;
  TestWriteStream test_stream;
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

  TestWriteStream test_stream;
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
}

BOOST_AUTO_TEST_CASE(PersistentConfigTest) {
  SizedPool<> pool;
  PersistentConfig config(&pool);
  BigStruct bs;
  config.Register(gsl::ensure_z("bigs"), &bs);
  OtherStruct os;
  config.Register(gsl::ensure_z("other"), &os);

  TestWriteStream test_stream;
  int count = 0;
  int error = 0;
  auto callback = [&](int err) { count++; error = err; };

  config.Command(gsl::ensure_z("bogus"), test_stream, callback);

  BOOST_CHECK_EQUAL(count, 1);
  BOOST_CHECK_EQUAL(error, 0);
  BOOST_CHECK_EQUAL(test_stream.ostr.str(), "unknown command\n");

  count = 0;
  test_stream.ostr.str("");
  BOOST_REQUIRE_EQUAL(os.vfloat, 3.5);
  config.Command(gsl::ensure_z("set other.vfloat 4.5"), test_stream, callback);

  BOOST_CHECK_EQUAL(count, 1);
  BOOST_CHECK_EQUAL(error, 0);
  BOOST_CHECK_EQUAL(test_stream.ostr.str(), "OK\n");
  BOOST_CHECK_EQUAL(os.vfloat, 4.5);

  count = 0;
  test_stream.ostr.str("");
  config.Command(gsl::ensure_z("get bigs.u8"), test_stream, callback);

  BOOST_CHECK_EQUAL(count, 1);
  BOOST_CHECK_EQUAL(error, 0);
  BOOST_CHECK_EQUAL(test_stream.ostr.str(), "3\n");

  count = 0;
  test_stream.ostr.str("");
  config.Command(gsl::ensure_z("enumerate"), test_stream, callback);

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
