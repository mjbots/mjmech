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

#include <boost/python.hpp>

#include "base/fast_stream.h"
#include "base/telemetry_format.h"

namespace {
namespace bp = boost::python;
using namespace mjmech::base;
typedef TelemetryFormat TF;

bp::object g_collections = bp::import("collections");
bp::object g_main_module = bp::import("__main__");
bp::object g_main_namespace = g_main_module.attr("__dict__");

template <typename Base>
class RecordingStream {
 public:
  RecordingStream(Base& base) : base_(base) {}

  void ignore(size_t length) {
    char buffer[length];
    read(buffer, length);
  }

  void read(char* buffer, size_t length) {
    base_.read(buffer, length);
    last_read_ = base_.gcount();
    ostr_.write(buffer, last_read_);
  }

  size_t gcount() const { return last_read_; }

  std::string str() const { return ostr_.str(); }

  FastOStringStream ostr_;
  size_t last_read_ = 0;
  Base& base_;
};

std::string FormatDict(bp::dict enum_items) {
  std::ostringstream ostr;
  ostr << "{";
  bp::list items = enum_items.items();
  for (int i = 0; i < bp::len(items); i++) {
    int key = bp::extract<int>(items[i][0]);
    std::string value = bp::extract<std::string>(items[i][1]);
    ostr << boost::format("%d:\"%s\",") % key % value;
  }
  ostr << "}";
  return ostr.str();
}

bp::object MakeEnumType(const std::string& field_name,
                        bp::dict enum_items) {
  bp::dict globals = bp::extract<bp::dict>(g_main_namespace);
  bp::dict locals;

  bp::exec(bp::str((boost::format(R"XX(
class %1%(int):
  __slots__ = ()
  __fields__ = ('enum_items')

  def __new__(_cls, value):
    if isinstance(value, str):
      if '(' in value:
        value = int(value.split('(', 1)[1].split(')', 1)[0])
      else:
        value = int(value)
    return int.__new__(_cls, value)

  enum_items = %2%

  def __repr__(self):
    if self in self.enum_items:
      return "'%%s(%%d)'" %% (self.enum_items[self], self)
    return '%%s' %% self
)XX") % field_name % FormatDict(enum_items)).str()),
                  globals, locals);

  return locals[field_name];
}

class ReadArchive {
 public:
  ReadArchive(const std::string& schema, const std::string& name)
      : schema_(schema), name_(name) {
    FastIStringStream schema_stream(schema_);
    TelemetryReadStream<FastIStringStream> schema_read_stream(schema_stream);

    root_ = ReadSchema(schema_read_stream, name_);
  }

  bp::object deserialize(const std::string& data) {
    FastIStringStream schema_stream(schema_);
    FastIStringStream data_stream(data);
    TelemetryReadStream<FastIStringStream> schema_read_stream(schema_stream);
    TelemetryReadStream<FastIStringStream> data_read_stream(data_stream);

    return ReadDataSchema(root_, schema_read_stream, data_read_stream);
  }

  bp::object root() const { return root_; }

 private:
  template <typename Stream>
  bp::object ReadSchema(Stream& schema, const std::string& name) {
    uint32_t flags = schema.template Read<uint32_t>();
    if (flags != 0) {
      throw SystemError::einval(
          (boost::format("unsupported SchemaFlags %d") % flags).str());
    }

    return ReadSchemaObject(schema, name, false);
  }

  template <typename Stream>
  bp::object ReadSchemaObject(Stream& schema, const std::string& name,
                              bool ignore) {
    uint32_t flags = schema.template Read<uint32_t>();
    if (flags != 0) {
      throw SystemError::einval(
          (boost::format("unsupported ObjectFlags %d") % flags).str());
    }

    bp::list fields_list;
    bp::list field_names;

    while (true) {
      const uint32_t field_flags = schema.template Read<uint32_t>();
      const std::string field_name = schema.ReadString();

      bp::object field = ReadFieldRecord(
          schema, field_flags, field_name, ignore);

      int type = bp::extract<int>(field["type"]);
      if (type == static_cast<int>(TF::FieldType::kFinal)) {
        break;
      }

      if (!ignore) {
        field_names.append(field["name"]);
        fields_list.append(field);
      }
    }

    if (!ignore) {
      bp::dict result;
      result["fields"] = fields_list;
      result["tuple"] = g_collections.attr("namedtuple")(
          name, field_names);

      return result;
    }

    return bp::object();
  }

  template <typename Stream>
  bp::object ReadFieldRecord(Stream& stream,
                             uint32_t field_flags,
                             const std::string& field_name,
                             bool ignore) {
    bp::dict result;

    uint32_t ft = stream.template Read<uint32_t>();
    result["type"] = ft;

    if (!ignore) {
      result["flags"] = field_flags;
      result["name"] = field_name;
    }

    switch (static_cast<TF::FieldType>(ft)) {
      case TF::FieldType::kBool:
      case TF::FieldType::kInt8:
      case TF::FieldType::kUInt8:
      case TF::FieldType::kInt16:
      case TF::FieldType::kUInt16:
      case TF::FieldType::kInt32:
      case TF::FieldType::kUInt32:
      case TF::FieldType::kInt64:
      case TF::FieldType::kUInt64:
      case TF::FieldType::kFloat32:
      case TF::FieldType::kFloat64:
      case TF::FieldType::kPtime:
      case TF::FieldType::kString: {
        // No type specific information.
        break;
      }
      case TF::FieldType::kPair: {
        bp::object child1 = ReadFieldRecord(
            stream, field_flags, "first", ignore);
        bp::object child2 = ReadFieldRecord(
            stream, field_flags, "second", ignore);
        if (!ignore) {
          bp::list children;
          children.append(child1);
          children.append(child2);
          result["children"] = children;
        }
        break;
      }
      case TF::FieldType::kArray: {
        uint32_t nelements = stream.template Read<uint32_t>();
        if (nelements >
            static_cast<uint32_t>(TF::BlockOffsets::kMaxBlockSize)) {
          throw SystemError::einval("corrupt array size");
        }
        bp::object child = ReadFieldRecord(
            stream, field_flags, field_name, ignore);
        if (!ignore) {
          bp::list children;
          children.append(child);
          result["nelements"] = nelements;
          result["children"] = children;
        }
        break;
      }
      case TF::FieldType::kVector: {
        bp::object child = ReadFieldRecord(
            stream, field_flags, field_name, ignore);
        if (!ignore) {
          bp::list children;
          children.append(child);
          result["children"] = children;
        }
        break;
      }
      case TF::FieldType::kObject: {
        bp::object child = ReadSchemaObject(stream, field_name, ignore);
        if (!ignore) {
          bp::list children;
          children.append(child);
          result["children"] = children;
        }
        break;
      }
      case TF::FieldType::kOptional: {
        bp::object child = ReadFieldRecord(
            stream, field_flags, field_name, ignore);
        if (!ignore) {
          bp::list children;
          children.append(child);
          result["children"] = children;
        }
        break;
      }
      case TF::FieldType::kEnum: {
        uint32_t size = stream.template Read<uint32_t>();
        if (ignore) {
          stream.Ignore(size);
        } else {
          uint32_t nvalues = stream.template Read<uint32_t>();
          bp::dict enum_items;
          for (uint32_t i = 0; i < nvalues; i++) {
            uint32_t key = stream.template Read<uint32_t>();
            std::string value = stream.ReadString();
            enum_items[key] = value;
          }
          result["enum_items"] = enum_items;
          result["enum_type"] = MakeEnumType(field_name, enum_items);
        }
        break;
      }
      case TF::FieldType::kFinal: {
        break;
      }
      default: {
        throw SystemError::einval(
            (boost::format("unimplemented field type %d") % ft).str());
      }
    }

    return result;
  }

  template <typename Stream>
  bp::object ReadDataSchema(bp::object schema_result,
                            Stream& schema, Stream& data) {
    uint32_t flags = schema.template Read<uint32_t>();
    if (flags != 0) {
      throw SystemError::einval(
          (boost::format("unsupported SchemaFlags %d") % flags).str());
    }

    return ReadDataObject(schema_result, schema, data);
  }

  template <typename Stream>
  bp::object ReadDataObject(bp::object schema_result,
                            Stream& schema, Stream& data) {
    uint32_t flags = schema.template Read<uint32_t>();
    if (flags != 0) {
      throw SystemError::einval(
          (boost::format("unsupported ObjectFlags %d") % flags).str());
    }

    int index = 0;
    bp::list elements;
    bp::list fields = bp::extract<bp::list>(schema_result["fields"]);

    while (true) {
      schema.template Read<uint32_t>(); // flags
      std::string name = schema.ReadString(); // name

      if (index >= bp::len(fields)) {
        uint32_t final_type = schema.template Read<uint32_t>();
        if (final_type != static_cast<uint32_t>(TF::FieldType::kFinal)) {
          throw SystemError::einval("corrupt log");
        }
        break;
      }

      bp::object field_result = fields[index];

      elements.append(ReadDataField(field_result, schema, data));
      index += 1;
    }

    return schema_result["tuple"](*bp::tuple(elements));
  }

  template <typename Stream>
  bp::object ReadDataField(bp::object field_result,
                           Stream& schema_stream, Stream& data_stream) {
    uint32_t ft = schema_stream.template Read<uint32_t>();

    switch (static_cast<TF::FieldType>(ft)) {
      case TF::FieldType::kBool: {
        return bp::object(
            data_stream.template Read<uint8_t>() ? true : false);
      }
      case TF::FieldType::kInt8: {
        return bp::object(
            static_cast<int>(data_stream.template Read<int8_t>()));
      }
      case TF::FieldType::kUInt8: {
        return bp::object(
            static_cast<int>(data_stream.template Read<uint8_t>()));
      }
      case TF::FieldType::kInt16: {
        return bp::object(data_stream.template Read<int16_t>());
      }
      case TF::FieldType::kUInt16: {
        return bp::object(data_stream.template Read<uint16_t>());
      }
      case TF::FieldType::kInt32: {
        return bp::object(data_stream.template Read<int32_t>());
      }
      case TF::FieldType::kUInt32: {
        return bp::object(data_stream.template Read<uint32_t>());
      }
      case TF::FieldType::kInt64: {
        return bp::object(data_stream.template Read<int64_t>());
      }
      case TF::FieldType::kUInt64: {
        return bp::object(data_stream.template Read<uint64_t>());
      }
      case TF::FieldType::kFloat32: {
        return bp::object(data_stream.template Read<float>());
      }
      case TF::FieldType::kFloat64: {
        return bp::object(data_stream.template Read<double>());
      }
      case TF::FieldType::kPtime: {
        return bp::object(data_stream.template Read<int64_t>() / 1e6);
      }
      case TF::FieldType::kString: {
        return bp::object(data_stream.ReadString());
      }
      case TF::FieldType::kPair: {
        bp::list children = bp::extract<bp::list>(field_result["children"]);
        bp::list elements;
        elements.append(
            ReadDataField(children[0], schema_stream, data_stream));
        elements.append(
            ReadDataField(children[1], schema_stream, data_stream));
        return bp::tuple(elements);
      }
      case TF::FieldType::kArray: {
        uint32_t nelements = schema_stream.template Read<uint32_t>();
        return ReadVariableSize(field_result, nelements,
                                schema_stream, data_stream);
      }
      case TF::FieldType::kVector: {
        uint32_t nelements = data_stream.template Read<uint32_t>();
        return ReadVariableSize(field_result, nelements,
                                schema_stream, data_stream);
      }
      case TF::FieldType::kObject: {
        bp::list children = bp::extract<bp::list>(field_result["children"]);
        return ReadDataObject(children[0], schema_stream, data_stream);
      }
      case TF::FieldType::kOptional: {
        uint8_t present = data_stream.template Read<uint8_t>();
        if (present != 0 && present != 1) {
          throw SystemError::einval("corrupt optional element");
        }
        bp::list list_result =
            ReadVariableSize(
                field_result, present, schema_stream, data_stream);
        if (!present) { return bp::object(); }
        return list_result[0];
      }
      case TF::FieldType::kEnum: {
        uint32_t size = schema_stream.template Read<uint32_t>();
        schema_stream.Ignore(size);

        return field_result["enum_type"](
            data_stream.template Read<uint32_t>());
      }
      case TF::FieldType::kFinal: {
        throw SystemError::einval("invalid state");
      }
    }
    throw SystemError::einval(
        (boost::format("invalid type %d") % ft).str());
  }

  template <typename Stream>
  bp::list ReadVariableSize(bp::object field_result,
                            uint32_t nelements,
                            Stream& schema_stream,
                            Stream& data_stream) {
    typedef decltype(schema_stream.stream()) RawStream;
    RecordingStream<RawStream> recording_raw_stream(schema_stream.stream());
    TelemetryReadStream<RecordingStream<RawStream> > recording_stream(
        recording_raw_stream);

    // First, consume the schema while recording, we don't care
    // about the result.
    ReadFieldRecord(
        recording_stream,
        bp::extract<uint32_t>(field_result["flags"]),
        bp::extract<std::string>(field_result["name"]),
        true);

    // Now we can read from the data stream as much as we want using
    // the recorded schema data.
    bp::list children = bp::extract<bp::list>(field_result["children"]);
    bp::object child = children[0];
    bp::list result;
    for (uint32_t i = 0; i < nelements; i++) {
      FastIStringStream sub_schema_raw_stream(recording_raw_stream.str());
      TelemetryReadStream<FastIStringStream> sub_schema_stream(
          sub_schema_raw_stream);
      result.append(ReadDataField(child, sub_schema_stream, data_stream));
    }

    return result;
  }

  const std::string schema_;
  const std::string name_;
  bp::object root_;
};
}

BOOST_PYTHON_MODULE(_telemetry_archive) {
  using namespace bp;

  class_<ReadArchive>("ReadArchive", init<std::string, std::string>())
      .def("deserialize", &ReadArchive::deserialize)
      .add_property("root", &ReadArchive::root)
      ;
}
