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

#include "static_function.h"

/// This class manages locks to the output stream.
class LockManager {
 public:
  // Lower number lock types have priority.
  enum LockType {
    kCommandManager,

    // Note, the following is only used for TelemetryManager initiated
    // messages, not responses to CommandManager requests.
    kTelemetryManager,
    kNumLockers,
  };

  typedef StaticFunction<void (int)> ReleaseCallback;
  typedef StaticFunction<void (ReleaseCallback)> LockCallback;

  /// @return true if the given lock type is already acquired.
  bool locked(LockType) const;

  /// Request that the given callback be invoked when the lock is next
  /// available.  It will be passed a callback that must be invoked
  /// when the lock should be released.
  ///
  /// It is an error to call this function if locked(type) == true.
  void Lock(LockType, LockCallback);

 private:
  void MaybeStart();

  bool outstanding_ = false;
  std::array<LockCallback, kNumLockers> callbacks_;
};
