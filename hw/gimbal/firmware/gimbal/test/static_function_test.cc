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

#include "gimbal/static_function.h"

#include <functional>

#include <boost/test/auto_unit_test.hpp>

BOOST_AUTO_TEST_CASE(BasicStaticFunctionTest) {
  int a = 0;

  StaticFunction<void ()> fun1;
  BOOST_CHECK_EQUAL(fun1.valid(), false);

  fun1 = [&]() { a--; };
  BOOST_CHECK_EQUAL(fun1.valid(), true);
  fun1();
  BOOST_CHECK_EQUAL(a, -1);

  a = 0;

  BOOST_CHECK_EQUAL(a, 0);
  StaticFunction<void ()> fun2 = [&]() { a++; };
  BOOST_CHECK_EQUAL(a, 0);
  fun2();
  BOOST_CHECK_EQUAL(a, 1);

  auto lambda3 = [&](int val) { a = val; };
  StaticFunction<void (int)> fun3 = lambda3;
  BOOST_CHECK_EQUAL(a, 1);

  fun3(10);
  BOOST_CHECK_EQUAL(a, 10);

  std::bind(fun3, 19)();
  BOOST_CHECK_EQUAL(a, 19);

  StaticFunction<void ()> fun4 = std::bind(lambda3, 11);
  BOOST_CHECK_EQUAL(a, 19);
  fun4();
  BOOST_CHECK_EQUAL(a, 11);

  // Try binding something into a StaticFunction.
  auto fun5 = fun3.shrink();
  fun5(15);
  BOOST_CHECK_EQUAL(a, 15);

  StaticFunction<void ()> fun6 = std::bind(fun5, 12);
  fun6();
  BOOST_CHECK_EQUAL(a, 12);
}
