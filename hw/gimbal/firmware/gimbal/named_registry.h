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

/// Associates a textual name with a pointer, up to a fixed maximum
/// number of elements.
template <typename T, std::size_t Size>
class NamedRegistryBase {
 public:
  struct Element {
    gsl::cstring_span name;
    T* ptr;
  };

  enum CreateMode {
    kAllowCreate,
    kFindOnly,
  };

  Element* FindOrCreate(const gsl::cstring_span& name,
                        CreateMode create_mode) {
    for (auto& element: elements_) {
      if (element.name.size() == 0) {
        switch (create_mode) {
          case kAllowCreate: {
            element.name = name;
            return &element;
          }
          case kFindOnly: {
            return nullptr;
          }
        }
      } else if (element.name == name) {
        return &element;
      }
    }
    // Whoops, we ran out of space.
    Expects(false);
    return nullptr;
  }

  Element& operator[](std::size_t index) {
    Expects(index < Size);
    return elements_[index];
  }

  const Element& operator[](std::size_t index) const {
    Expects(index < Size);
    return elements_[index];
  }

  std::size_t size() const { return Size; }

  std::array<Element, Size> elements_;
};
