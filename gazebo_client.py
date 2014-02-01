#!/usr/bin/python

# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

'''This file implements a python interface to the gazebo simulation's
network server.'''

import eventlet
import math
import time

import msg.gz_string_pb2
import msg.gz_string_v_pb2
import msg.packet_pb2
import msg.publishers_pb2
import msg.subscribe_pb2

class ParseError(RuntimeError):
    pass

class Publisher(object):
    def __init__(self):
        self.topic = None
        self.msg_type = None
        self._listeners = []

    def publish(self, msg):
        self._publish_impl(msg)

    def _publish_impl(self, message):
        for connection in self._listeners:
            connection.write(message)

    def _connect(self, connection):
        self._listeners.append(connection)

    def remove(self):
        # TODO
        pass

class Subscriber(object):
    def __init__(self):
        # TODO
        self.topic = None
        self.msg_type = None
        self.callback = None

    def remove(self):
        # TODO
        pass

class Connection(object):
    def __init__(self):
        self.address = None
        self.socket = None
        self._local_host = None
        self._local_port = None

    def connect(self, address):
        print 'Connection.connect'
        self.address = address
        self.socket = eventlet.connect(self.address)

    def serve(self, callback):
        self.socket = eventlet.listen(('', 0))
        self._local_host, self._local_port = self.socket.getsockname()
        eventlet.serve(self.socket, callback)

    def read(self):
        print 'Connection.read'
        header = self.socket.recv(8)
        if len(header) < 8:
            return None

        try:
            size = int(header, 16)
        except ValueError:
            raise ParseError('invalid header: ' + header)

        data = self.socket.recv(size)
        if len(data) < size:
            return None

        packet = msg.packet_pb2.Packet.FromString(data)
        return packet

    def write(self, message):
        while self.socket is None:
            eventlet.sleep(0.1)

        data = message.SerializeToString()

        header = '%08X' % len(data)
        self.socket.send(header + data)

    @property
    def local_host(self):
        while self._local_host is None:
            eventlet.sleep(0.1)
        return self._local_host

    @property
    def local_port(self):
        while self._local_port is None:
            eventlet.sleep(0.1)
        return self._local_port

class _PublisherRecord(object):
    def __init__(self, msg):
        self.topic = msg.topic
        self.msg_type = msg.msg_type
        self.host = msg.host
        self.port = msg.port

class Manager(object):
    '''The Manager communicates with the gazebo server and provides
    the top level interface for communicating with it.'''

    def __init__(self, address):
        '''@param address - a tuple of (host, port)'''
        self.address = address
        self.master = Connection()
        self.server = Connection()
        self.namespaces = []
        self.publisher_records = set()
        self.publishers = {}

    def run(self):
        '''Call to start the connection and process events.  It is
        expected to be invoked from an eventlet spawn, and will only
        return when the connection is closed or an error occurs.'''
        print 'Manager.run'
        self.master.connect(self.address)
        eventlet.spawn_n(self.server.serve, self._handle_server_connection)

        # Read and process the required three initialization packets.
        initData = self.master.read()
        if initData.type != 'version_init':
            raise ParseError('unexpected initialization packet: ' +
                             initData.type)
        self._handle_version_init(
            msg.gz_string_pb2.GzString.FromString(initData.serialized_data))

        namespacesData = self.master.read()
        if namespacesData.type != 'topic_namepaces_init':
            raise ParseError('unexpected namespaces init packet: ' +
                             namespacesData.type)
        self._handle_topic_namespaces_init(
            msg.gz_string_v_pb2.GzString_V.FromString(
                namespacesData.serialized_data))

        publishersData = self.master.read()
        if publishersData.type != 'publishers_init':
            raise ParseError('unexpected publishers init packet: ' +
                             publishersData.type)
        self._handle_publishers_init(
            msg.publishers_pb2.Publishers.FromString(
                publishersData.serialized_data))

        print 'Connection: initialized!'
        self.initialized = True

        # Enter the normal message dispatch loop.
        while True:
            data = self.master.read()
            if data is None:
                return

            self._process_message(data)

    def advertise(self, topic_name, msg_type):
        '''Prepare to publish a topic.

        @returns a Publisher instance'''
        if topic_name in self.publishers:
            raise RuntimeError('multiple publishers for: ' + topic_name)

        to_send = msg.publish_pb2.Publish()
        to_send.topic = topic_name
        to_send.msg_type = msg_type
        to_send.host = self.server.local_host
        to_send.port = self.server.local_port

        self._write_packet('advertise', to_send)
        result = Publisher()
        result.topic = topic_name
        result.msg_type = msg_type
        self.publishers[topic_name] = result

        return result

    def subscribe(self, topic_name, msg_type, callback):
        '''Request to receive updates on a topic.

        @returns a Subscriber instance'''
        assert False
        pass

    def _write_packet(self, name, message):
        packet = msg.packet_pb2.Packet()
        cur_time = time.time()
        packet.stamp.sec = int(cur_time)
        packet.stamp.nsec = int(math.fmod(cur_time, 1) * 1e9)
        packet.type = name
        packet.serialized_data = message.SerializeToString()
        self.master.write(packet)

    def _handle_server_connection(self, socket, remote_address):
        this_connection = Connection()
        this_connection.socket = socket

        while True:
            message = this_connection.read()
            if message is None:
                return
            if message.type == "sub":
                self._handle_server_sub(
                    this_connection,
                    msg.subscribe_pb2.Subscribe.FromString(
                        message.serialized_data))
            else:
                print 'Manager.handle_server_connection unknown msg:', msg.type

    def _handle_server_sub(self, this_connection, msg):
        if not msg.topic in self.publishers:
            print 'Manager.handle_server_sub unknown topic:', msg.topic
            return

        publisher = self.publishers[msg.topic]
        if publisher.msg_type != msg.msg_type:
            print ('Manager.handle_server_sub type mismatch ' +
                   'requested=%d publishing=%s') % (
                publisher.msg_type, msg.msg_type)
            return

        publisher._connect(this_connection)

    def _process_message(self, packet):
        print 'Manager.process_message', packet
        if packet.type in Manager._MSG_HANDLERS:
            handler, packet_type = Manager._MSG_HANDLERS[packet.type]
            handler(self, packet_type.FromString(packet.serialized_data))
        else:
            print 'unhandled message type: ' + packet.type

    def _handle_version_init(self, msg):
        print 'Manager.handle_version_init' + msg.data
        version = float(msg.data.split(' ')[1])
        if version < 2.2:
            raise ParseError('Unsupported gazebo version: ' + msg.data)

    def _handle_topic_namespaces_init(self, msg):
        self.namespaces = msg.data
        print 'Manager.handle_topic_namespaces_init', self.namespaces

    def _handle_publishers_init(self, msg):
        print 'Manager.handle_publishers_init'
        for publisher in msg.publisher:
            self.publisher_records.add(_PublisherRecord(publisher))
            print '  %s - %s %s:%d' % (publisher.topic, publisher.msg_type,
                                       publisher.host, publisher.port)

    def _handle_publisher_add(self, msg):
        print 'Manager.handle_publisher_add: %s - %s %s:%d' % (
            msg.topic, msg.msg_type, msg.host, msg.port)
        self.publisher_records.add(_PublisherRecord(msg))

    def _handle_publisher_del(self, msg):
        print 'Manager.handle_publisher_del', msg.topic
        self.publisher_records.remove(_PublisherRecord(msg))

    def _handle_namespace_add(self, msg):
        print 'Manager.handle_namespace_add', msg.data
        self.namespaces.append(msg.data)

    _MSG_HANDLERS = {
        'publisher_add' : (_handle_publisher_add, msg.publish_pb2.Publish),
        'publisher_del' : (_handle_publisher_del, msg.publish_pb2.Publish),
        'namespace_add' : (_handle_namespace_add, msg.gz_string_pb2.GzString),
        }

import msg.joint_cmd_pb2

if __name__ == '__main__':
    manager = Manager(('localhost', 11345))
    eventlet.spawn_n(manager.run)
    joint_cmd_publisher = manager.advertise(
        '/gazebo/default/mj_mech/joint_cmd',
        'gazebo.msgs.JointCmd')

    joint_cmd = msg.joint_cmd_pb2.JointCmd()
    joint_cmd.name = 'mj_mech::coxa_hinge1'
    joint_cmd.axis = 0
    joint_cmd.position.target = 0
    joint_cmd.position.p_gain = 5.0
    joint_cmd.position.i_gain = 0.01
    joint_cmd.position.d_gain = 2.0

    eventlet.sleep(0.1)

    joints = []
    for x in range(1, 5):
        joints += ['body_hinge%d' % x,
                   'coxa_hinge%d' % x,
                   'femur_hinge%d' % x]

    while True:
        for name in joints:
            joint_cmd.name = 'mj_mech::' + name
            joint_cmd_publisher.publish(joint_cmd)
        eventlet.sleep(1)


    eventlet.sleep(100)
