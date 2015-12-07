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

#include "gimbal/herkulex_protocol.h"

#include <map>

#include <boost/test/auto_unit_test.hpp>

#include "stream_test.h"

namespace {
class TestOperations : public HerkulexProtocol::Operations {
 public:
  virtual ~TestOperations() {}

  uint8_t address() const override { return 0xfd; }

  void WriteRam(uint8_t address, uint8_t value) override {
    file_[address] = value;
  }

  uint8_t ReadRam(uint8_t address) override {
    return 0xff - address;
  }

  void Reboot() override { reboot_ = true; }

  std::map<uint8_t, uint8_t> file_;
  bool reboot_ = false;
};
}

BOOST_AUTO_TEST_CASE(BasicHerkulexProtocolTest) {
  TestOperations operations;
  test::TestStream stream;
  SizedPool<> pool;
  HerkulexProtocol dut(pool, stream, operations);

  int count = 0;
  int error = 0;
  dut.AsyncStart([&](int err) { count++; error = err; });

  BOOST_CHECK_EQUAL(count, 1);
  BOOST_CHECK_EQUAL(error, 0);

  // Send a reboot command because that is easy.
  BOOST_REQUIRE_EQUAL(operations.reboot_, false);
  stream.Write(std::string("\xff\xff\x07\xfd\x09\xf2\x0c", 7));
  BOOST_REQUIRE_EQUAL(operations.reboot_, true);
  operations.reboot_ = false;
  BOOST_CHECK_EQUAL(stream.ostr.str(), "");

  // Send a RAM write and verify it happened.
  BOOST_CHECK_EQUAL(operations.file_.size(), 0u);
  stream.Write(std::string("\xff\xff\x0a\xfd\x03\xc0\x3e\x35\x01\x01", 10));
  BOOST_CHECK_EQUAL(operations.reboot_, false);
  BOOST_CHECK_EQUAL(operations.file_.count(0x35), 1);
  BOOST_CHECK_EQUAL(operations.file_[0x35], 0x01);
  BOOST_CHECK_EQUAL(stream.ostr.str(), "");

  // And now a RAM read.
  stream.Write(std::string("\xff\xff\x09\xfd\x04\xc4\x3a\x35\x01", 9));
  std::string actual = stream.ostr.str();
  stream.ostr.str("");
  BOOST_CHECK_EQUAL(
      actual,
      std::string("\xff\xff\x0c\xfd\x44\x4a\xb4\x35\x01\xca\x00\x00", 12));
}
