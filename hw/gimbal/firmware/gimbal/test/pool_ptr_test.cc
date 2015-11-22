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

#include "gimbal/pool_ptr.h"

#include <boost/test/auto_unit_test.hpp>

namespace {
struct TestStruct {
  int a = 1;
  int b = 2;
  int c = 3;
  double f = 9.0;
};

class TestClass {
 public:
  int* a = nullptr;

  ~TestClass() {
    if (a) { *a = 2; }
  }
};
}

BOOST_AUTO_TEST_CASE(BasicPoolPtrTest) {
  SizedPool<> pool;
  PoolPtr<int> foo(&pool);

  *foo = 0;
  BOOST_CHECK_EQUAL(*foo, 0);

  // Verify that the constructor is called.
  PoolPtr<TestStruct> foo2(&pool);
  BOOST_CHECK_EQUAL(foo2->a, 1);
  BOOST_CHECK_EQUAL(foo2->b, 2);

  // And verify that destructors are called.
  int my_int = 9;
  BOOST_CHECK_EQUAL(my_int, 9);

  {
    PoolPtr<TestClass> foo3(&pool);
    foo3->a = &my_int;
    BOOST_CHECK_EQUAL(my_int, 9);
  }

  BOOST_CHECK_EQUAL(my_int, 2);
}
