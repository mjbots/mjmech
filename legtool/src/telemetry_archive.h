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

#include <boost/assert.hpp>

#include "telemetry_format.h"
#include "visit_archive.h"

namespace legtool {
/// Generate schema and binary records according to the structure
/// defined in telemetry_format.h.
template <typename RootSerializable>
class TelemetryWriteArchive {
 public:
  typedef TelemetryFormat TF;

  TelemetryWriteArchive() : schema_(MakeSchema()) {}

  std::string schema() const { return schema_; }

  std::string Serialize(const RootSerializable* serializable) const {
    std::ostringstream ostr;
    TelemetryWriteStream stream(ostr);
    DataVisitor visitor(stream);
    visitor.Accept(const_cast<RootSerializable*>(serializable));
    return ostr.str();
  }

  static std::string MakeSchema() {
    std::ostringstream ostr;
    TelemetryWriteStream stream(ostr);

    stream.Write(static_cast<uint32_t>(0)); // SchemaFlags

    WriteSchemaObject(stream, static_cast<RootSerializable*>(0));

    return ostr.str();
  }

 private:
  template <typename Serializable>
  static void WriteSchemaObject(TelemetryWriteStream& stream,
                                Serializable* serializable) {
    SchemaVisitor visitor(stream);
    visitor.Accept(serializable);

    visitor.Finish();
  }

  template <typename T>
  struct FakeNvp {
    FakeNvp(T* value) : value_(value) {}
    T* value() const { return value_; }

    T* const value_;
  };

  class SchemaVisitor : public VisitArchive<SchemaVisitor> {
   public:
    SchemaVisitor(TelemetryWriteStream& stream) : stream_(stream) {
      stream.Write(static_cast<uint32_t>(0)); // ObjectFlags
    }

    void Finish() {
      // Write out the "final" record.
      stream_.Write(static_cast<uint32_t>(0));
      stream_.Write(std::string());
      stream_.Write(static_cast<uint32_t>(TF::FieldType::kFinal));
    }

    template <typename NameValuePair>
    void Visit(const NameValuePair& pair) {
      stream_.Write(static_cast<uint32_t>(0)); // FieldFlags;
      stream_.Write(std::string(pair.name()));

      VisitArchive<SchemaVisitor>::Visit(pair);
    }

    template <typename NameValuePair>
    void VisitSerializable(const NameValuePair& pair) {
      stream_.Write(static_cast<uint32_t>(TF::FieldType::kObject));

      SchemaVisitor visitor(stream_);
      visitor.Accept(pair.value());
      visitor.Finish();
    }

    template <typename NameValuePair>
    void VisitScalar(const NameValuePair& pair) {
      VisitHelper(pair, pair.value(), 0);
    }

   private:
    template <typename NameValuePair, typename First, typename Second>
    void VisitHelper(const NameValuePair& pair,
                     std::pair<First, Second>*,
                     int) {
      stream_.Write(static_cast<uint32_t>(TF::FieldType::kPair));

      VisitArchive<SchemaVisitor>::Visit(
          FakeNvp<First>(&pair.value()->first));
      VisitArchive<SchemaVisitor>::Visit(
          FakeNvp<Second>(&pair.value()->second));
    }

    template <typename NameValuePair, typename T, std::size_t N>
    void VisitHelper(const NameValuePair& pair,
                     std::array<T, N>*,
                     int) {
      stream_.Write(static_cast<uint32_t>(TF::FieldType::kArray));
      stream_.Write(static_cast<uint32_t>(N));

      VisitArchive<SchemaVisitor>::Visit(
          FakeNvp<T>(static_cast<T*>(nullptr)));
    }

    template <typename NameValuePair, typename T>
    void VisitHelper(const NameValuePair& pair,
                     std::vector<T>*,
                     int) {
      stream_.Write(static_cast<uint32_t>(TF::FieldType::kVector));

      VisitArchive<SchemaVisitor>::Visit(
          FakeNvp<T>(static_cast<T*>(nullptr)));
    }

    template <typename NameValuePair, typename T>
    void VisitHelper(const NameValuePair& pair,
                     boost::optional<T>*,
                     int) {
      stream_.Write(static_cast<uint32_t>(TF::FieldType::kOptional));

      VisitArchive<SchemaVisitor>::Visit(FakeNvp<T>(static_cast<T*>(0)));
    }

    template <typename NameValuePair, typename T>
    void VisitHelper(const NameValuePair& pair,
                     T*,
                     long) {
      VisitPrimitive(pair);
    }

    template <typename NameValuePair>
    void VisitPrimitive(const NameValuePair& pair) {
      stream_.Write(static_cast<uint32_t>(FindType(pair.value())));
    }


    TelemetryWriteStream& stream_;
  };

  class DataVisitor : public VisitArchive<DataVisitor> {
   public:
    DataVisitor(TelemetryWriteStream& stream) : stream_(stream) {}

    template <typename NameValuePair>
    void VisitSerializable(const NameValuePair& pair) {
      DataVisitor sub(stream_);
      sub.Accept(pair.value());
    }

    template <typename NameValuePair>
    void VisitScalar(const NameValuePair& pair) {
      VisitHelper(pair, pair.value(), 0);
    }

   private:
    template <typename NameValuePair, typename First, typename Second>
    void VisitHelper(const NameValuePair& pair,
                     std::pair<First, Second>*,
                     int) {
      VisitArchive<DataVisitor>::Visit(
          FakeNvp<First>(&pair.value()->first));
      VisitArchive<DataVisitor>::Visit(
          FakeNvp<Second>(&pair.value()->second));
    }

    template <typename NameValuePair, typename T, std::size_t N>
    void VisitHelper(const NameValuePair& pair,
                     std::array<T, N>* value,
                     int) {
      for (int i = 0; i < N; i++) {
        VisitArchive<DataVisitor>::Visit(
            FakeNvp<T>(&(*value)[i]));
      }
    }

    template <typename NameValuePair, typename T>
    void VisitHelper(const NameValuePair& pair,
                     std::vector<T>* value,
                     int) {
      stream_.Write(static_cast<uint32_t>(value->size()));
      for (int i = 0; i < static_cast<int>(value->size()); i++) {
        VisitArchive<DataVisitor>::Visit(
            FakeNvp<T>(&(*value)[i]));
      }
    }

    template <typename NameValuePair, typename T>
    void VisitHelper(const NameValuePair& pair,
                     boost::optional<T>* value,
                     int) {
      stream_.Write(static_cast<uint8_t>(*value ? 1 : 0));
      if (*value) {
        VisitArchive<DataVisitor>::Visit(
            FakeNvp<T>(&(**value)));
      }
    }

    template <typename NameValuePair, typename T>
    void VisitHelper(const NameValuePair& pair,
                     T*,
                     long) {
      VisitPrimitive(pair);
    }

    template <typename NameValuePair>
    void VisitPrimitive(const NameValuePair& pair) {
      stream_.Write(*pair.value());
    }

    TelemetryWriteStream& stream_;
  };

  static TF::FieldType FindType(bool*) { return TF::FieldType::kBool; }
  static TF::FieldType FindType(int8_t*) { return TF::FieldType::kInt8; }
  static TF::FieldType FindType(uint8_t*) { return TF::FieldType::kUInt8; }
  static TF::FieldType FindType(int16_t*) { return TF::FieldType::kInt16; }
  static TF::FieldType FindType(uint16_t*) { return TF::FieldType::kUInt16; }
  static TF::FieldType FindType(int32_t*) { return TF::FieldType::kInt32; }
  static TF::FieldType FindType(uint32_t*) { return TF::FieldType::kUInt32; }
  static TF::FieldType FindType(int64_t*) { return TF::FieldType::kInt64; }
  static TF::FieldType FindType(uint64_t*) { return TF::FieldType::kUInt64; }
  static TF::FieldType FindType(float*) { return TF::FieldType::kFloat32; }
  static TF::FieldType FindType(double*) { return TF::FieldType::kFloat64; }
  static TF::FieldType FindType(std::string*) { return TF::FieldType::kString; }
  static TF::FieldType FindType(boost::posix_time::ptime*) {
    return TF::FieldType::kPtime;
  }

  const std::string schema_;
};
}
