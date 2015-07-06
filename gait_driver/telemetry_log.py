# Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.

import capnp
import random
import struct
import tempfile

# The telemetry log has the following format:
#
# HEADER: TLOG0001
#
# The body consists of zero or more "blocks"
#
# Each block consists of:
#    4 byte block type
#    4 byte size (all bytes after these first 8)
_BLOCK_STRUCT = struct.Struct('II')

# A pstring is a 4 byte size followed by that many bytes of UTF8
# string.
_PSTRING_STRUCT = struct.Struct('I')

# Block type 1: Description
#   4 byte flags
#   4 byte identifier
#   pstring NAME
#   pstring capnproto message name
#   entirety of capnproto schema file
_BLOCK_DESCRIPTION = 1
_DESCRIPTION_STRUCT = struct.Struct('III')


# Block type 2: Data
#   4 byte flags
#   4 byte identifier
#   serialized capnproto structure
_BLOCK_DATA = 2
_DATA_STRUCT = struct.Struct('II')


_HEADER = 'TLOG0001'


def _parse_struct(compiled_struct, data):
    if len(data) < compiled_struct.size:
        raise RuntimeError('malformed data')

    this_data = data[0:compiled_struct.size]
    rest = data[compiled_struct.size:]

    result = compiled_struct.unpack(this_data)
    return result, rest


class Writer(object):
    '''Writes a telemetry log to the given file like object.'''
    def __init__(self, fd):
        self._fd = fd
        self._fd.write(_HEADER)

        self._record = {}
        self._names = {}
        self._nextid = 0

    def register(self, name, capnproto_schema, struct_name):
        '''Returns a function which can be called to write new data to
        the log.  The function takes a single capnproto object.'''

        if name in self._names:
            raise RuntimeError('name already registered')

        self._names[name] = True

        this_id = self._nextid
        self._nextid += 1
        record = _Record(self, this_id, name, capnproto_schema, struct_name)

        self._record[this_id] = record
        self._write_description(record)

        return record

    def close(self):
        '''Close the log.'''
        self._fd.flush()
        self._fd = None

    def _write_description(self, record):
        data = (_DESCRIPTION_STRUCT.pack(
                0, record.identifier, len(record.name)) +
                record.name.encode('utf8') +
                _PSTRING_STRUCT.pack(len(record.struct_name)) +
                record.struct_name.encode('utf8') + record.schema)
        self._write_block(_BLOCK_DESCRIPTION, data)

    def _write_data(self, identifier, stuff):
        data = (_DATA_STRUCT.pack(0, identifier) + stuff)
        self._write_block(_BLOCK_DATA, data)

    def _write_block(self, block_type, data):
        block = _BLOCK_STRUCT.pack(block_type, len(data)) + data
        # TODO jpieper: Worry about doing this in a background thread
        # or asynchronously so that we don't screw up real-time
        # things.
        self._fd.write(block)


class _Record(object):
    def __init__(self, writer, identifier, name, schema, struct_name):
        self.writer = writer
        self.identifier = identifier
        self.name = name
        self.schema = schema
        self.struct_name = struct_name

    def __call__(self, thing):
        # TODO jpieper: Assert that "thing" is of the correct
        # capnproto type.
        self.writer._write_data(self.identifier, thing.to_bytes())


class BulkReader(object):
    def __init__(self, fd):
        self._fd = fd
        self._fd.seek(0, 0)
        header = self._fd.read(8)
        if header != _HEADER:
            raise RuntimeError('header does not match')

    def _read_blocks(self):
        '''Iterate over all blocks in the telemetry log.'''
        self._fd.seek(len(_HEADER), 0)

        while True:
            block_header = self._fd.read(_BLOCK_STRUCT.size)
            if len(block_header) < _BLOCK_STRUCT.size:
                return

            (block_type, block_size), _ = _parse_struct(
                _BLOCK_STRUCT, block_header)

            block_data = self._fd.read(block_size)
            if len(block_data) != block_size:
                # An incomplete block, we'll call that done too.
                break

            yield block_type, block_data

    def _parse_description(self, block_data, filter=None):
        '''Returns a tuple of (identifier, _BulkRecord)'''

        (flags, identifier, namelen), rest = _parse_struct(
            _DESCRIPTION_STRUCT, block_data)

        if len(rest) < namelen:
            raise RuntimeError('malformed data')

        name = rest[0:namelen]

        if not (filter is None or
                isinstance(filter, list) and name in filter or
                filter(name)):
            return None, None

        rest = rest[namelen:]
        (struct_name_len,), rest = _parse_struct(_PSTRING_STRUCT, rest)
        struct_name = rest[0:struct_name_len]
        rest = rest[struct_name_len:]

        return identifier, _BulkRecord(identifier, name, rest, struct_name)

    def records(self):
        '''Return a dictionary mapping record names to an empty
        element of each record type.'''

        result = {}

        for block_type, block_data in self._read_blocks():
            if block_type != _BLOCK_DESCRIPTION:
                continue

            identifier, bulk_record = self._parse_description(block_data)
            result[bulk_record.name] = bulk_record.message.new_message()

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
            if block_type == _BLOCK_DESCRIPTION:
                identifier, bulk_record = self._parse_description(
                    block_data, filter=filter)
                if identifier is None:
                    continue
                records[identifier] = bulk_record
            elif block_type == _BLOCK_DATA:
                (flags, identifier), rest = _parse_struct(
                    _DATA_STRUCT, block_data)
                if identifier not in records:
                    continue

                records[identifier].add(rest)

        return dict((x.name, x.elements) for x in records.itervalues())


def _rewrite_schema_id(schema):
    '''Take a capnproto schema, and rewrite its ID to be a new random
    one so as to avoid collisions.'''

    # For now, we only handle schemas where the ID is the very first
    # thing.
    assert schema.startswith('@0x')
    result = schema[:]

    new_id = random.getrandbits(63) | (1 << 63)

    result = '@0x%08x' % new_id + result[19:]

    return result


class _BulkRecord(object):
    def __init__(self, identifier, name, schema, struct_name):
        self.identifier = identifier
        self.name = name
        self.schema = schema

        # Rewrite the file level ID in the schema so that the upper
        # bit is not set, this way capnproto won't yell about
        # duplicate IDs.  Stupid libraries with unobservable global
        # state mumble mumble...

        rewritten_schema = _rewrite_schema_id(schema)

        tf = tempfile.NamedTemporaryFile()
        tf.write(rewritten_schema)
        tf.flush()

        self.cp = capnp.load(tf.name)
        self.message = getattr(self.cp, struct_name)

        self.elements = []

    def add(self, data):
        self.elements.append(self.message.from_bytes(data))
