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

'''Read files stored in the format as described in
telemetry_format.h.'''

import snappy
import cStringIO as stringio
import struct

import telemetry_archive


_BLOCK_STRUCT = struct.Struct('<HI')
_HEADER = 'TLOG0002'
_BLOCK_SCHEMA = 1
_BLOCK_DATA = 2


class BlockDataFlags(object):
    kPreviousOffset = 1 << 0
    kSchemaCRC = 1 << 1
    kSnappy = 1 << 8


class BulkReader(object):
    '''Read many records from a log.'''

    def __init__(self, fd):
        self._fd = fd
        self._fd.seek(0, 0)
        header = self._fd.read(8)

        assert header == _HEADER

    def _read_blocks(self):
        '''Iterate over all blocks, yielding their data.'''
        self._fd.seek(len(_HEADER), 0)

        while True:
            block_header = self._fd.read(_BLOCK_STRUCT.size)
            if len(block_header) < _BLOCK_STRUCT.size:
                return

            (block_type, block_size) = _BLOCK_STRUCT.unpack(block_header)

            block_data = self._fd.read(block_size)
            if len(block_data) != block_size:
                # An incomplete read, we'll call that done too.
                break

            yield block_type, block_data

    def _parse_schema(self, block_data, filter=None):
        stream = telemetry_archive.ReadStream(stringio.StringIO(block_data))
        identifier = stream.read_uint32()
        flags = stream.read_uint32()
        name = stream.read_pstring()
        schema = stream.stream.read()

        if not (filter is None or
                isinstance(filter, list) and name in filter or
                filter(name)):
            return None, None

        return identifier, _BulkRecord(identifier, flags, name, schema)

    def records(self):
        '''Return a dictionary mapping record names to an empty
        element of each record type.'''

        result = {}

        for block_type, block_data in self._read_blocks():
            if block_type != _BLOCK_SCHEMA:
                continue

            identifier, bulk_record = self._parse_schema(block_data)
            result[bulk_record.name] = bulk_record.make_empty()

        return result

    def get(self, filter=None):
        '''Return a dictionary containing lists of objects
        corresponding to one or more records.

        If "filter" is present, and is a list, it is a set of records
        to return.  If it is a callable, it should be a predicate
        which returns true if the named record should be included.
        '''

        records = {}

        for block_type, block_data in self._read_blocks():
            if block_type == _BLOCK_SCHEMA:
                identifier, bulk_record = self._parse_schema(
                    block_data, filter=filter)
                if identifier is None:
                    continue
                records[identifier] = bulk_record
            elif block_type == _BLOCK_DATA:
                stream = telemetry_archive.ReadStream(
                    stringio.StringIO(block_data))
                identifier = stream.read_uint32()
                flags = stream.read_uint16()

                if identifier not in records:
                    continue

                if flags & BlockDataFlags.kPreviousOffset:
                    flags &= ~(BlockDataFlags.kPreviousOffset)
                    _ = stream.read_varint()
                if flags & BlockDataFlags.kSchemaCRC:
                    assert False  # not supported yet
                if flags & BlockDataFlags.kSnappy:
                    flags &= ~(BlockDataFlags.kSnappy)
                    rest = stream.stream.read()
                    stream = telemetry_archive.ReadStream(
                        stringio.StringIO(snappy.uncompress(rest)))
                assert flags == 0  # no unknown flags

                rest = stream.stream.read()
                records[identifier].add(rest)

        return dict((x.name, x.elements) for x in records.itervalues())


class _BulkRecord(object):
    def __init__(self, identifier, flags, name, schema):
        self.identifier = identifier
        self.flags = flags
        self.name = name
        self.schema = schema
        self.archive = telemetry_archive.ReadArchive(schema, name)
        self.elements = []

    def add(self, data):
        self.elements.append(self.archive.deserialize(data))

    def make_empty(self):
        return self.archive.root.tuple()
