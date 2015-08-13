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

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>
#include <boost/signals2/signal.hpp>

#include "comm.h"
#include "common.h"
#include "signal_result.h"
#include "visitor.h"

namespace legtool {

template <class Factory>
class HerkuleXProtocol : boost::noncopyable {
 public:
  struct Packet {
    int servo = 0;
    int command = 0;
    std::string data;
    bool cksum_good = false;
  };

  typedef boost::optional<Packet> OptionalPacket;

  struct StatusResponse : public Packet {
    StatusResponse(const Packet& packet)
        : StatusResponse(packet, packet.data) {}

    StatusResponse(const Packet& packet,
                   const std::string& status_data)
        : Packet(packet),
          reg48(status_data.at(0)),
          reg49(status_data.at(1)) {}

    uint8_t reg48 = 0;
    uint8_t reg49 = 0;

    bool exceeded_input_voltage_limit = (reg48 & 0x01) != 0;
    bool exceeded_allowed_pot_limit = (reg48 & 0x02) != 0;
    bool exceeded_temperature_limit = (reg48 & 0x04) != 0;
    bool invalid_packet = (reg48 & 0x08) != 0;
    bool overload_detected = (reg48 & 0x10);
    bool driver_fault_detected = (reg48 & 0x20);
    bool eep_reg_distorted = (reg48 & 0x40);

    bool moving = (reg49 & 0x01) != 0;
    bool inposition = (reg49 & 0x02) != 0;
    bool checksum_error = (reg49 & 0x04) != 0;
    bool unknown_command = (reg49 & 0x08) != 0;
    bool exceeded_reg_range = (reg49 & 0x10) != 0;
    bool garbage_detected = (reg49 & 0x20) != 0;
    bool motor_on = (reg49 & 0x40) != 0;
  };

  HerkuleXProtocol(boost::asio::io_service& service,
                   Factory& factory)
      : service_(service),
        factory_(factory) {}

  struct Parameters {
    typename Factory::Parameters stream;
    double packet_timeout_s = 0.2;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(LT_NVP(stream));
      a->Visit(LT_NVP(packet_timeout_s));
    }
  };

  Parameters* parameters() { return &parameters_; }

  template <typename Handler>
  BOOST_ASIO_INITFN_RESULT_TYPE(Handler,
                                void(boost::system::error_code))
  AsyncStart(Handler handler) {
    boost::asio::detail::async_result_init<
        Handler,
        void (boost::system::error_code)> init(
            BOOST_ASIO_MOVE_CAST(Handler)(handler));

    factory_.AsyncCreate(
        parameters_.stream,
        std::bind(&HerkuleXProtocol::HandleStart, this, init.handler,
                  std::placeholders::_1,
                  std::placeholders::_2));

    return init.result.get();
  }

  template <typename Handler>
  BOOST_ASIO_INITFN_RESULT_TYPE(Handler,
                                void(boost::system::error_code))
  SendPacket(const Packet& packet,
             Handler handler) {
    boost::asio::detail::async_result_init<
        Handler,
        void (boost::system::error_code)> init(
            BOOST_ASIO_MOVE_CAST(Handler)(handler));

    BOOST_ASSERT(packet.data.size() < (216 - 7));
    BOOST_ASSERT(packet.servo >= 0 && packet.servo <= 0xfe);
    BOOST_ASSERT(packet.command >= 0 && packet.command <= 0xff);

    const uint8_t packet_size = 7 + packet.data.size();

    const uint8_t cksum1 =
        Checksum1(packet_size, packet.servo, packet.command, packet.data);

    buffer_[0] = 0xff;
    buffer_[1] = 0xff;
    buffer_[2] = packet_size;
    buffer_[3] = packet.servo;
    buffer_[4] = packet.command;
    buffer_[5] = cksum1 & 0xfe;
    buffer_[6] = (cksum1 ^ 0xff) & 0xfe;
    std::memcpy(&buffer_[7], packet.data.data(), packet.data.size());

    boost::asio::async_write(
        *stream_, boost::asio::buffer(buffer_, 7 + packet.data.size()),
        std::bind(init.handler, std::placeholders::_1));

    return init.result.get();
  }

  /// Receive a single packet, or raise a TimeoutError if no packet is
  /// received within @param timeout_s seconds.
  template <typename Handler>
  BOOST_ASIO_INITFN_RESULT_TYPE(Handler,
                                void(boost::system::error_code, Packet))
  ReceivePacket(Handler handler) {
    return SignalResult::Wait(
        service_, &read_signal_,
        parameters_.packet_timeout_s, handler);
  }

  template <typename Handler>
  BOOST_ASIO_INITFN_RESULT_TYPE(Handler,
                                void(boost::system::error_code, Packet))
  SendReceivePacket(const Packet& to_send, Handler handler) {
    boost::asio::detail::async_result_init<
        Handler,
        void (boost::system::error_code)> init(
            BOOST_ASIO_MOVE_CAST(Handler)(handler));

    SendPacket(to_send, [=](const boost::system::error_code& ec) {
        if (ec) {
          init.handler(ec, Packet());
          return;
        }

        ReceivePacket(init.handler);
      });

    return init.result.get();
  }

 private:
  void HandleStart(ErrorHandler handler,
                   const boost::system::error_code& ec,
                   SharedStream stream) {
    if (ec) {
      service_.post(std::bind(handler, ec));
      return;
    }

    BOOST_ASSERT(!stream_);
    stream_ = stream;

    boost::asio::spawn(
        service_, ErrorWrap(std::bind(&HerkuleXProtocol::ReadLoop, this,
                                      std::placeholders::_1)));

    service_.post(std::bind(handler, boost::system::error_code()));
  }

  void ReadLoop(boost::asio::yield_context yield) {
    char data_buffer[256] = {};
    boost::asio::streambuf streambuf(512);
    while (true) {
      // Read until we get a frame header.
      boost::asio::async_read_until(
          *stream_, streambuf, "\xff\xff", yield);

      // Clear out the streambuf until we have our delimeter.
      std::istream istr(&streambuf);
      std::string buf;
      while (buf != "\xff\xff") {
        char c = 0;
        istr.read(&c, 1);
        if (istr.gcount() != 1) {
          throw std::runtime_error("inconsistent header");
        }
        buf = buf + std::string(&c, 1);
        if (buf.size() > 2) {
          buf = buf.substr(buf.size() - 2);
        }
      }

      // Read the header.
      if (streambuf.size() < 5) {
        boost::asio::async_read(
            *stream_, streambuf,
            boost::asio::transfer_at_least(5 - streambuf.size()),
            yield);
      }

      char header[5] = {};
      istr.read(header, 5);
      if (istr.gcount() != 5) {
        throw std::runtime_error("inconsistent header");
      }

      uint8_t size = header[0];
      uint8_t servo = header[1];
      uint8_t command = header[2];
      uint8_t cksum1 = header[3];
      uint8_t cksum2 = header[4];

      if (size < 7) {
        // Malformed header.  Skip back to looking for
        // synchronization.  Probably should log?

        // TODO jpieper: Think about logging.
        continue;
      }

      if (streambuf.size() < (size - 7)) {
        boost::asio::async_read(
            *stream_,
            streambuf,
            boost::asio::transfer_at_least(
                (size - 7) - streambuf.size()),
            yield);
      }

      istr.read(data_buffer, size - 7);
      std::string data(data_buffer, size - 7);

      uint8_t expected_cksum1 = Checksum1(size, servo, command, data);
      uint8_t expected_cksum2 = expected_cksum1 ^ 0xff;

      Packet result;
      result.servo = servo;
      result.command = command;
      result.data = data;
      result.cksum_good = (cksum1 == (expected_cksum1 & 0xfe) &&
                           cksum2 == (expected_cksum2 & 0xfe));

      read_signal_(&result);
    }
  }

  void SendPacket(boost::asio::yield_context yield,
                  int servo, int command, const std::string& data) {
    BOOST_ASSERT(servo >= 0 && servo <= 0xfe);
    BOOST_ASSERT(command >= 0 && command <= 0xff);

    int packet_size = data.size() + 7;
    uint8_t cksum1 = Checksum1(packet_size, servo, command, data);
    uint8_t cksum2 = cksum1 ^ 0xff;

    uint8_t header[] = {
      0xff,
      0xff,
      packet_size,
      servo,
      command,
      cksum1 & 0xfe,
      cksum2 & 0xfe,
    };

    ConstBufferSequence buffers{
      boost::asio::buffer(header), boost::asio::buffer(data)};
    boost::asio::async_write(*stream_, buffers, yield);
  }

  static uint8_t Checksum1(uint8_t size, uint8_t servo, uint8_t cmd,
                           const std::string& data) {
    return size ^ servo ^ cmd ^
        std::accumulate(data.begin(), data.end(), 0,
                        [](uint8_t a, uint8_t b) { return a ^ b; });
  }

  boost::asio::io_service& service_;
  Factory& factory_;
  Parameters parameters_;
  SharedStream stream_;
  boost::signals2::signal<void (const Packet*)> read_signal_;
  char buffer_[256];
};

template <class Factory>
class HerkuleX : public HerkuleXProtocol<Factory> {
 public:
  typedef HerkuleXProtocol<Factory> Base;

  template <typename... Args>
  HerkuleX(Args&&... args)
      : HerkuleXProtocol<Factory>(std::forward<Args>(args)...) {}

  enum Command {
    EEP_WRITE = 0x01,
    EEP_READ = 0x02,
    RAM_WRITE = 0x03,
    RAM_READ = 0x04,
    I_JOG = 0x05,
    S_JOG = 0x06,
    STAT = 0x07,
    ROLLBACK = 0x08,
    REBOOT = 0x09,
    ACK_EEP_WRITE = 0x41,
    ACK_EEP_READ = 0x42,
    ACK_RAM_WRITE = 0x43,
    ACK_RAM_READ = 0x44,
    ACK_I_JOG = 0x45,
    ACK_S_JOG = 0x46,
    ACK_STAT = 0x47,
    ACK_ROLLBACK = 0x48,
    ACK_REBOOT = 0x49,
  };

  enum MagicID {
    BROADCAST = 0xfe,
  };

  struct MemReadResponse : public Base::StatusResponse {
    MemReadResponse(const typename Base::Packet& packet)
        : Base::StatusResponse(packet,
                               packet.data.substr(packet.data.size() - 2)),
          register_start(packet.data.at(0)),
          length(packet.data.at(1)) {}

    uint8_t register_start = 0;
    uint8_t length = 0;
  };

  MemReadResponse MemRead(uint8_t command, uint8_t servo,
                          uint8_t reg, uint8_t length,
                          boost::asio::yield_context yield) {
    typename Base::Packet to_send;
    to_send.servo = servo;
    to_send.command = command;
    char buf[2] = { reg, length };
    to_send.data = std::string(buf, 2);

    typename Base::Packet result = SendReceivePacket(to_send, yield);
    if (result.servo != to_send.servo && result.servo != BROADCAST) {
      throw std::runtime_error(
          (boost::format("Synchronization error, sent request for servo 0x%02x, "
                         "response from 0x%02x") %
           static_cast<int>(to_send.servo) %
           static_cast<int>(result.servo)).str());
    }

    if (result.command != (0x40 | command)) {
      throw std::runtime_error(
          (boost::format("Expected response command 0x%02x, received 0x%02x") %
           static_cast<int>(0x40 | command) %
           static_cast<int>(result.command)).str());
    }

    if (result.data.size() != length + 4) {
      throw std::runtime_error(
          (boost::format("Expected length response %d, received %d") %
           static_cast<int>(length + 4) %
           result.data.size()).str());
    }

    MemReadResponse response = result;

    if (response.register_start != reg) {
      throw std::runtime_error(
          (boost::format("Expected register 0x%02x, received 0x%02x") %
           static_cast<int>(reg) %
           static_cast<int>(response.register_start)).str());
    }

    if (response.length != length) {
      throw std::runtime_error(
          (boost::format("Expected length %d, received %d") %
           static_cast<int>(length) %
           static_cast<int>(response.length)).str());
    }

    return response;
  }
};
}

