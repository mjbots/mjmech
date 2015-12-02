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

#include "gimbal/command_manager.h"

#include <boost/test/auto_unit_test.hpp>

#include "gimbal/lock_manager.h"
#include "stream_test.h"

BOOST_AUTO_TEST_CASE(CommandManagerTest) {
  SizedPool<> pool;
  test::TestStream stream;
  LockManager lock_manager;

  CommandManager dut(pool, stream, lock_manager);
  std::string dut_message = "";
  ErrorCallback dut_callback;
  dut.Register(gsl::ensure_z("test1"),
               [&](const gsl::cstring_span& message, ErrorCallback cbk) {
                 dut_message = std::string(message.data(), message.size());
                 dut_callback = cbk;
               });

  BOOST_CHECK_EQUAL(stream.read_callback_.valid(), false);

  int count = 0;
  int error = 0;
  dut.AsyncStart([&](int err) { count++; error = err; });
  BOOST_CHECK_EQUAL(count, 0);

  dut.Poll();

  BOOST_CHECK_EQUAL(count, 1);
  BOOST_CHECK_EQUAL(error, 0);
  BOOST_CHECK_EQUAL(stream.read_callback_.valid(), true);

  stream.Write("foobaz\n");
  BOOST_CHECK_EQUAL(dut_message, "");
  BOOST_CHECK_EQUAL(dut_callback.valid(), false);
  std::string actual = stream.ostr.str();
  BOOST_CHECK_EQUAL(actual, std::string("unknown command\r\n"));
  BOOST_CHECK_EQUAL(lock_manager.locked(LockManager::kCommandManager), false);

  stream.ostr.str("");
  stream.Write("test1 sample command\n");

  BOOST_CHECK_EQUAL(dut_callback.valid(), true);
  BOOST_CHECK_EQUAL(dut_message, "sample command");
  BOOST_CHECK_EQUAL(lock_manager.locked(LockManager::kCommandManager), true);
  dut_callback(0);
  BOOST_CHECK_EQUAL(lock_manager.locked(LockManager::kCommandManager), false);


}
