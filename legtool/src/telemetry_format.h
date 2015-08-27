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

#include <boost/date_time/posix_time/posix_time_types.hpp>

#include "common.h"

/// @file
///
/// Common definitions:
///    pstring
///      * uint32_t length
///      * length bytes of binary data
///    endian-ness
///      * all values are stored little-endian
///
/// TelemetryLog:
///   * TLOG0002
///   * N Block s
///
/// Block
///   * uint16_t - BlockType
///   * uint32_t - size
///   * size bytes of data
///
/// BlockType
///   1 - BlockSchema
///   2 - BlockData
///
/// BlockSchema
///  * uint32_t identifier
///  * BlockSchemaFlags
///  * pstring name
///  * Schema
///
/// BlockData
///  * uint32_t identifier
///  * BlockDataFlags
///  * DataObject
///
/// Schema
///  * uint32_t SchemaFlags
///  * SchemaObject
///
/// SchemaFlags
///  none currently defined
///
/// SchemaObject
///  * uint32_t ObjectFlags
///  * N FieldRecord s (final must be type "final")
///
/// ObjectFlags
///  none currently defined
///
/// FieldRecord
///  * uint32_t FieldFlags
///  * pstring
///  * uint32_t FieldType
///  * field specific data
///
/// FieldFlags
///  none currently defined
///
/// FieldType
///  0 - final
///  1 - bool
///  2 - int8_t
///  3 - uint8_t
///  4 - int16_t
///  5 - uint16_t
///  6 - int32_t
///  7 - uint32_t
///  8 - int64_t
///  9 - uint64_t
///  10 - float32_t
///  11 - float64_t
///  12 - boost::posix_time::ptime (int64_t)
///  13 - string
///  14 - pair
///     * field specific data
///       * FieldType first
///         * type-specific data
///       * FieldType second
///         * type-specific data/
///  15 - array
///     * field specific data
///       * uint32_t nelements
///       * FieldType
///         * type-specified data
///  16 - vector
///     * field specific data
///       * FieldType
///         * type-specified data
///  17 - object
///     * field specific data
///         * SchemaObject
///  18 - optional
///     * field specific data
///       * FieldType
///         * type-specified data
///  19 - enum
///     * field specific data
///       * uint32_t nvalues
///       * nvalues copies of
///         * uint32_t value
///         * pstring
///
/// DataObject
///  * N DataRecord s
///
/// DataRecord
///  * bool - uint8_t
///  * *_t - *_t
///  * boost::posix_time::ptime - int64_t (microseconds since epoch)
///  * string - pstring
///  * pair
///    * first - DataObject
///    * second - DataObject
///  * array
///    * N DataRecords
///  * vector
///    * uint32_t nelements
///    * nelements DataObject s
///  * object
///    * Object
///  * optional
///    * uint8_t - present
///    * (optionally) DataObject
///  * enum
///    * uint32_t - value

namespace legtool {

struct TelemetryFormat {
  static constexpr const char* kHeader = "TLOG0002";

  enum class BlockType {
    kBlockSchema = 1,
    kBlockData = 2,
  };

  enum class BlockSchemaFlags {
  };

  enum class BlockDataFlags {
    // TODO jpieper: The following flags could be useful.
    //
    //  * SchemaCRC: Include a CRC of the schema which generated this
    //    data block for consistency checking.
    //  * Previous offset: Record an offset to the previous record of
    //    this identifier.
  };

  enum class SchemaFlags {
  };

  enum class ObjectFlags {
  };

  enum class FieldFlags {
  };

  enum class FieldType {
    kFinal = 0,
    kBool,
    kInt8,
    kUInt8,
    kInt16,
    kUInt16,
    kInt32,
    kUInt32,
    kInt64,
    kUInt64,
    kFloat32,
    kFloat64,
    kPtime,
    kString,
    kPair,
    kArray,
    kVector,
    kObject,
    kOptional,
    kEnum,
  };

  enum class BlockOffsets {
    kBlockType = 0,
    kBlockSize = 2,
    kBlockData = 6,
    kMaxBlockSize = 1 << 24,
  };
};

template <typename Stream>
class TelemetryWriteStream {
 public:
  typedef TelemetryFormat TF;

  TelemetryWriteStream(Stream& ostr) : ostr_(ostr) {}

  void Write(const std::string& data) {
    if (data.size() >
        static_cast<std::size_t>(TF::BlockOffsets::kMaxBlockSize)) {
      throw std::runtime_error("invalid pstring size");
    }
    Write(static_cast<uint32_t>(data.size()));
    RawWrite(data.data(), data.size());
  }

  void Write(bool value) {
    uint8_t to_write = value ? 1 : 0;
    Write(to_write);
  }

  void Write(int8_t value) { WriteScalar(value); }
  void Write(uint8_t value) { WriteScalar(value); }
  void Write(int16_t value) { WriteScalar(value); }
  void Write(uint16_t value) { WriteScalar(value); }
  void Write(int32_t value) { WriteScalar(value); }
  void Write(uint32_t value) { WriteScalar(value); }
  void Write(int64_t value) { WriteScalar(value); }
  void Write(uint64_t value) { WriteScalar(value); }
  void Write(float value) { WriteScalar(value); }
  void Write(double value) { WriteScalar(value); }
  void Write(const boost::posix_time::ptime time) {
    int64_t value = ConvertPtimeToMicroseconds(time);
    Write(value);
  }

  void RawWrite(const char* data, uint32_t size) {
    ostr_.write(data, size);
  }

 private:
  template <typename T>
  void WriteScalar(T value) {
#ifndef __ORDER_LITTLE_ENDIAN__
#error "Only little endian architectures supported"
#endif
    // TODO jpieper: Support big-endian.
    RawWrite(reinterpret_cast<const char*>(&value), sizeof(value));
  }

  Stream& ostr_;
};

class TelemetryReadStream {
 public:
  typedef TelemetryFormat TF;

  TelemetryReadStream(std::istream& istr) : istr_(istr) {}
  std::istream& stream() { return istr_; }

  template <typename T>
  inline T Read() {
    return ReadScalar<T>();
  }

 private:
  template <typename T>
  T ReadScalar() {
    T result = T();
    RawRead(reinterpret_cast<char*>(&result), sizeof(result));
    return result;
  }

  void RawRead(char* out, std::size_t size) {
    istr_.read(out, size);
    if (static_cast<std::size_t>(istr_.gcount()) != size) {
      throw std::runtime_error("EOF");
    }
  }

  std::istream& istr_;
};

template <>
inline boost::posix_time::ptime
TelemetryReadStream::Read<boost::posix_time::ptime>() {
  int64_t value = Read<int64_t>();
  return ConvertMicrosecondsToPtime(value);
}

template <>
inline std::string
TelemetryReadStream::Read<std::string>() {
  uint32_t size = Read<uint32_t>();
  if (size > static_cast<std::size_t>(TF::BlockOffsets::kMaxBlockSize)) {
    throw std::runtime_error("corrupt pstring");
  }
  std::string result(size, static_cast<char>(0));
  RawRead(&result[0], size);
  return result;
}

}
