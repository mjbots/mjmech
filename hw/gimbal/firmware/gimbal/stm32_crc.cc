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

#include "crc.h"

#include <boost/crc.hpp>

//#include "Inc/crc.h"

uint32_t CalculateCrc(const char* start, std::size_t length) {
  boost::crc_32_type crc;
  crc.process_bytes(start, length);
  return crc.checksum();

  // NOTE jpieper 2016-01-24: For some reason, which I don't feel like
  // debugging, the HAL accelerated version of this gives random
  // results on large data.  Instead, we just use the boost CRC
  // version above, which is correct, if slower.

  // return HAL_CRC_Calculate(
  //     &hcrc,
  //     const_cast<uint32_t*>(reinterpret_cast<const uint32_t*>(start)),
  //     length);
}
