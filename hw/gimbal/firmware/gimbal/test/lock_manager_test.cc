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

#include "gimbal/lock_manager.h"

#include <boost/test/auto_unit_test.hpp>

BOOST_AUTO_TEST_CASE(LockManagerTest) {
  LockManager dut;
  BOOST_CHECK_EQUAL(dut.locked(LockManager::kCommandManager), false);
  BOOST_CHECK_EQUAL(dut.locked(LockManager::kTelemetryManager), false);

  LockManager::ReleaseCallback current_release;
  auto do_work = [&](LockManager::ReleaseCallback release) {
    current_release = release;
  };

  BOOST_CHECK_EQUAL(current_release.valid(), false);
  dut.Lock(LockManager::kCommandManager, do_work);
  BOOST_CHECK_EQUAL(dut.locked(LockManager::kCommandManager), true);
  BOOST_CHECK_EQUAL(dut.locked(LockManager::kTelemetryManager), false);
  BOOST_CHECK_EQUAL(current_release.valid(), true);

  current_release(0);
  BOOST_CHECK_EQUAL(dut.locked(LockManager::kCommandManager), false);
  BOOST_CHECK_EQUAL(dut.locked(LockManager::kTelemetryManager), false);
}

BOOST_AUTO_TEST_CASE(LockManagerContentionTest) {
  LockManager dut;
  LockManager::ReleaseCallback com_release;
  LockManager::ReleaseCallback tel_release;
  auto com_work = [&](LockManager::ReleaseCallback cbk) { com_release = cbk; };
  auto tel_work = [&](LockManager::ReleaseCallback cbk) { tel_release = cbk; };

  dut.Lock(LockManager::kTelemetryManager, tel_work);
  BOOST_CHECK_EQUAL(dut.locked(LockManager::kTelemetryManager), true);
  BOOST_CHECK_EQUAL(dut.locked(LockManager::kCommandManager), false);
  BOOST_CHECK_EQUAL(tel_release.valid(), true);
  BOOST_CHECK_EQUAL(com_release.valid(), false);

  dut.Lock(LockManager::kCommandManager, com_work);
  BOOST_CHECK_EQUAL(dut.locked(LockManager::kTelemetryManager), true);
  BOOST_CHECK_EQUAL(dut.locked(LockManager::kCommandManager), true);
  BOOST_CHECK_EQUAL(tel_release.valid(), true);
  BOOST_CHECK_EQUAL(com_release.valid(), false);

  tel_release(0);
  BOOST_CHECK_EQUAL(dut.locked(LockManager::kTelemetryManager), false);
  BOOST_CHECK_EQUAL(dut.locked(LockManager::kCommandManager), true);
  BOOST_CHECK_EQUAL(tel_release.valid(), true);
  BOOST_CHECK_EQUAL(com_release.valid(), true);

  com_release(0);
  BOOST_CHECK_EQUAL(dut.locked(LockManager::kTelemetryManager), false);
  BOOST_CHECK_EQUAL(dut.locked(LockManager::kCommandManager), false);
  BOOST_CHECK_EQUAL(tel_release.valid(), true);
  BOOST_CHECK_EQUAL(com_release.valid(), true);
}

