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

#include "base/gsl/gsl-lite.h"

#include "async_types.h"
#include "static_function.h"

class AsyncReadStream {
 public:
  AsyncReadStream() {}
  AsyncReadStream(const AsyncReadStream&) = delete;
  virtual ~AsyncReadStream() {}
  virtual void AsyncReadSome(const gsl::string_span&, SizeCallback) = 0;
};

class AsyncWriteStream {
 public:
  AsyncWriteStream() {}
  AsyncWriteStream(const AsyncWriteStream&) = delete;
  virtual ~AsyncWriteStream() {}
  virtual void AsyncWriteSome(const gsl::cstring_span&, SizeCallback) = 0;
};

class AsyncStream : public AsyncReadStream, public AsyncWriteStream {
 public:
  AsyncStream() {}
  AsyncStream(const AsyncStream&) = delete;
};

template <typename Stream>
void AsyncWrite(Stream& stream, const gsl::cstring_span& data,
                ErrorCallback callback) {
  if (data.empty()) { callback(0); return; }

  stream.AsyncWriteSome(
      data,
      [stream=&stream, data, cbk=callback.shrink<4>()]
      (ErrorCode error, std::size_t size) {
        if (error) { cbk(error); return; }
        if (data.size() == size) { cbk(0); return; }
        AsyncWrite(*stream,
                   gsl::cstring_span(data.begin() + size, data.end()),
                   cbk);
      });
}
