# Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import collections
import cStringIO as stringio
import os
import struct
import sys

SCRIPT_PATH = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(SCRIPT_PATH, 'build-x86_64'))

from _telemetry_archive import *


class ReadArchivePython(object):
    '''Parse schema and binary data objects serialized using the
    Telemetry protocol.

    collections.namedtuple classes are created at initialization time,
    then individual deserialized objects are instances of those
    classes so as to be reasonably efficient even when many data
    objects are deserialized.
    '''

    def __init__(self, schema, name):
        self.schema = schema
        stream = ReadStream(stringio.StringIO(self.schema))
        self.root = self._read_schema(stream, name)

    def deserialize(self, data):
        '''Deserialize data encoded using the constructed schema,
        returning namedtuple instances.'''
        schema_stream = ReadStream(stringio.StringIO(self.schema))
        data_stream = ReadStream(stringio.StringIO(data))

        return self._read_data_schema(self.root, schema_stream, data_stream)

    def _read_schema(self, stream, name):
        flags = stream.read_uint32()
        assert flags == 0

        return self._read_schema_object(stream, name)

    def _read_schema_object(self, stream, name):
        flags = stream.read_uint32()
        assert flags == 0

        result = {}

        result['fields'] = []

        while True:
            field_flags = stream.read_uint32()
            field_name = stream.read_pstring()

            field = self._read_field_record(stream, field_flags, field_name)

            if field['type'] == Format.kFinal:
                break

            result['fields'].append(field)

        result['tuple'] = collections.namedtuple(
            name, [x['name'] for x in result['fields']])
        return result

    def _read_field_record(self, stream, field_flags, field_name):
        result = {}

        result['flags'] = field_flags
        result['name'] = field_name
        result['type'] = stream.read_uint32()

        ft = result['type']

        if (ft == Format.kBool or
            ft == Format.kInt8 or
            ft == Format.kUInt8 or
            ft == Format.kInt16 or
            ft == Format.kUInt16 or
            ft == Format.kInt32 or
            ft == Format.kUInt32 or
            ft == Format.kInt64 or
            ft == Format.kUInt64 or
            ft == Format.kFloat32 or
            ft == Format.kFloat64 or
            ft == Format.kPtime or
            ft == Format.kString):
            # There isn't any type specific information here.
            pass
        elif ft == Format.kPair:
            child1 = self._read_field_record(stream, field_flags, 'first')
            child2 = self._read_field_record(stream, field_flags, 'second')
            result['children'] = [child1, child2]
        elif ft == Format.kArray:
            nelements = stream.read_uint32()
            if nelements > Format.kMaxBlockSize:
                raise RuntimeError('corrupt array size')
            result['nelements'] = nelements
            result['children'] = [
                self._read_field_record(stream, field_flags, field_name)]
        elif ft == Format.kVector:
            result['children'] = [
                self._read_field_record(stream, field_flags, field_name)]
        elif ft == Format.kObject:
            result['children'] = [
                self._read_schema_object(stream, field_name)]
        elif ft == Format.kOptional:
            result['children'] = [
                self._read_field_record(stream, field_flags, field_name)]
        elif ft == Format.kEnum:
            raise NotImplementedError()
        elif ft == Format.kFinal:
            pass
        else:
            raise RuntimeError('unimplemented field type %d' % ft)

        return result

    def _read_data_schema(self, schema_result, schema_stream, data_stream):
        flags = schema_stream.read_uint32()
        assert flags == 0

        return self._read_data_object(
            schema_result, schema_stream, data_stream)

    def _read_data_object(self, schema_result, schema_stream, data_stream):
        flags = schema_stream.read_uint32()
        assert flags == 0

        index = 0
        elements = []
        while True:
            _ = schema_stream.read_uint32()  # flags
            _ = schema_stream.read_pstring()  # name

            if index >= len(schema_result['fields']):
                final_type = schema_stream.read_uint32()
                assert final_type == Format.kFinal
                break

            field_result = schema_result['fields'][index]

            elements.append(
                self._read_data_field(
                    field_result, schema_stream, data_stream))
            index += 1

        return schema_result['tuple'](*elements)

    def _read_data_field(self, field_result, schema_stream, data_stream):
        _ = schema_stream.read_uint32()  # field_type

        ft = field_result['type']

        if ft == Format.kBool:
            return data_stream.read_bool()
        elif ft == Format.kInt8:
            return data_stream.read_int8()
        elif ft == Format.kUInt8:
            return data_stream.read_uint8()
        elif ft == Format.kInt16:
            return data_stream.read_int16()
        elif ft == Format.kUInt16:
            return data_stream.read_uint16()
        elif ft == Format.kInt32:
            return data_stream.read_int32()
        elif ft == Format.kUInt32:
            return data_stream.read_uint32()
        elif ft == Format.kInt64:
            return data_stream.read_int64()
        elif ft == Format.kUInt64:
            return data_stream.read_uint64()
        elif ft == Format.kFloat32:
            return data_stream.read_float32()
        elif ft == Format.kFloat64:
            return data_stream.read_float64()
        elif ft == Format.kPtime:
            return (data_stream.read_int64()) / 1e6
        elif ft == Format.kString:
            return data_stream.read_pstring()
        elif ft == Format.kPair:
            return (
                self._read_data_field(
                    field_result['children'][0], schema_stream, data_stream),
                self._read_data_field(
                    field_result['children'][1], schema_stream, data_stream))
        elif ft == Format.kArray:
            _ = schema_stream.read_uint32()  # nelements
            return self._read_variable_size(
                field_result, field_result['nelements'],
                schema_stream, data_stream)
        elif ft == Format.kVector:
            nelements = data_stream.read_uint32()
            return self._read_variable_size(
                field_result, nelements, schema_stream, data_stream)
        elif ft == Format.kObject:
            return self._read_data_object(
                field_result['children'][0], schema_stream, data_stream)
        elif ft == Format.kOptional:
            present = data_stream.read_uint8()
            assert present == 0 or present == 1
            list_result = self._read_variable_size(
                field_result, present, schema_stream, data_stream)
            if len(list_result) == 0:
                return None
            return list_result[0]
        elif ft == Format.kEnum:
            raise NotImplementedError()
        elif ft == Format.kFinal:
            pass

    def _read_variable_size(self, field_result, nelements,
                            schema_stream, data_stream):
        recording_raw_stream = RecordingStream(schema_stream.stream)
        recording_stream = ReadStream(recording_raw_stream)

        # First, consume the schema while recording, we don't care
        # about the result.
        self._read_field_record(
            recording_stream, field_result['flags'], field_result['name'])

        # Now we can read from the data stream as much as we want
        # using the recorded schema data.
        result = []
        for i in range(nelements):
            sub_schema_stream = ReadStream(stringio.StringIO(
                    recording_raw_stream.str()))
            result.append(self._read_data_field(
                    field_result['children'][0],
                    sub_schema_stream, data_stream))

        return result


class Format(object):
    kBlockSchema = 1
    kBlockData = 2

    kFinal = 0
    kBool = 1
    kInt8 = 2
    kUInt8 = 3
    kInt16 = 4
    kUInt16 = 5
    kInt32 = 6
    kUInt32 = 7
    kInt64 = 8
    kUInt64 = 9
    kFloat32 = 10
    kFloat64 = 11
    kPtime = 12
    kString = 13
    kPair = 14
    kArray = 15
    kVector = 16
    kObject = 17
    kOptional = 18
    kEnum = 19

    kMaxBlockSize = (1 << 24)


class ReadStream(object):
    '''Provides methods to read the native types of Telemetry
    formatted schema and data streams.'''
    def __init__(self, stream):
        self.stream = stream

    I8 = struct.Struct('<b')
    U8 = struct.Struct('<B')
    I16 = struct.Struct('<h')
    U16 = struct.Struct('<H')
    I32 = struct.Struct('<i')
    U32 = struct.Struct('<I')
    I64 = struct.Struct('<q')
    U64 = struct.Struct('<Q')
    F32 = struct.Struct('<f')
    F64 = struct.Struct('<d')

    def read_bool(self):
        return self.U8.unpack(self.stream.read(1))[0] != 0

    def read_int8(self):
        return self.I8.unpack(self.stream.read(1))[0]

    def read_uint8(self):
        return self.U8.unpack(self.stream.read(1))[0]

    def read_int16(self):
        return self.I16.unpack(self.stream.read(2))[0]

    def read_uint16(self):
        return self.U16.unpack(self.stream.read(2))[0]

    def read_int32(self):
        return self.I32.unpack(self.stream.read(4))[0]

    def read_uint32(self):
        return self.U32.unpack(self.stream.read(4))[0]

    def read_int64(self):
        return self.I64.unpack(self.stream.read(8))[0]

    def read_uint64(self):
        return self.U64.unpack(self.stream.read(8))[0]

    def read_float32(self):
        return self.F32.unpack(self.stream.read(4))[0]

    def read_float64(self):
        return self.F64.unpack(self.stream.read(8))[0]

    def read_pstring(self):
        size = self.read_uint32()
        return self.stream.read(size)

    def read_varint(self):
        result = 0
        position = 0
        while True:
            value = self.read_uint32()
            result |= (value & 0x7fffffff) << position
            position += 31

            if value < 0x80000000:
                break;
        return result


class RecordingStream(object):
    '''Approximates the interface of a stream, but records everything
    that is read for later perusal.'''
    def __init__(self, source):
        self.source = source
        self._data = stringio.StringIO()

    def read(self, size):
        result = self.source.read(size)
        self._data.write(result)
        return result

    def str(self):
        return self._data.getvalue()
