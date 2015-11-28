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

#include "async_stream.h"

class UsbCdcStream : public AsyncStream {
 public:
  UsbCdcStream();
  virtual ~UsbCdcStream();

  virtual void AsyncReadSome(const gsl::string_span&, SizeCallback) override;
  virtual void AsyncWriteSome(const gsl::cstring_span&, SizeCallback) override;

  void PollMillisecond();

  // These are not for public consumption.
  int8_t Init();
  int8_t Deinit();
  int8_t HandleControl(uint8_t, uint8_t*, uint16_t);
  int8_t HandleReceive(uint8_t*, uint32_t*);
  int8_t HandleTxComplete();

 private:
  uint8_t rx_buffer_[64] = {};
  std::size_t rx_position_ = 0;
  std::size_t rx_size_ = 0;
  gsl::string_span rx_queue_;
  SizeCallback rx_callback_;

  uint8_t tx_buffer_[64] = {};
  std::size_t tx_position_ = 0;

  gsl::cstring_span tx_queue_;
  SizeCallback tx_callback_;
  std::size_t tx_callback_size_ = 0;
  bool write_outstanding_ = false;

};
