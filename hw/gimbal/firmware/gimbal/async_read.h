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

#include <assert.h>
#include <cstring>

#include "async_stream.h"

struct AsyncReadUntilContext {
  AsyncReadStream* stream = nullptr;
  gsl::string_span buffer;
  SizeCallback callback;
  const char* delimiters = nullptr;
};

namespace detail {
inline void AsyncReadUntilHelper(AsyncReadUntilContext& context,
                                 uint16_t position) {
  auto handler =
      [ctx=&context,
       position] (ErrorCode error, std::size_t size) {
    if (error) {
      ctx->callback(error, position + size);
      return;
    }

    if (std::strchr(ctx->delimiters,
                    ctx->buffer.data()[position]) != nullptr) {
      ctx->callback(0, position + size);
      return;
    }

    if (position + 1 == static_cast<int>(ctx->buffer.size())) {
      // We overfilled our buffer without getting a terminator.
      ctx->callback(1001, position);
      return;
    }

    AsyncReadUntilHelper(*ctx, position + 1);
  };

  context.stream->AsyncReadSome(
      gsl::string_span(context.buffer.data() + position,
                       context.buffer.data() + position + 1), handler);
}
}

inline void AsyncReadUntil(AsyncReadUntilContext& context) {
  assert(context.buffer.size() < std::numeric_limits<uint16_t>::max());
  detail::AsyncReadUntilHelper(context, 0);
}

inline void AsyncIgnoreUntil(AsyncReadUntilContext& context) {
  context.stream->AsyncReadSome(
      gsl::string_span(context.buffer.data(), context.buffer.data() + 1),
      [ctx=&context](int error, std::size_t size) {
        if (error) {
          ctx->callback(error, 0);
          return;
        }

        if (std::strchr(ctx->delimiters,
                        ctx->buffer.data()[0]) != nullptr) {
          ctx->callback(0, 0);
          return;
        }

        AsyncIgnoreUntil(*ctx);
      });
}
