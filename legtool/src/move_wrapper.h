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

#pragma once

namespace legtool {
template <typename F>
struct move_wrapper : F
{
  move_wrapper(F&& f) : F(std::move(f)) {}

  move_wrapper(move_wrapper&&) = default;
  move_wrapper& operator=(move_wrapper&&) = default;

  move_wrapper(const move_wrapper&);
  move_wrapper& operator=(const move_wrapper&);
};

template <typename T>
auto move_handler(T&& t) -> move_wrapper<typename std::decay<T>::type>
{
  return std::move(t);
}
}
