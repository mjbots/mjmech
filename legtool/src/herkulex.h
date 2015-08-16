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
        void (boost::system::error_code, Packet)> init(
            BOOST_ASIO_MOVE_CAST(Handler)(handler));

    SendPacket(to_send, [=](const boost::system::error_code& ec) mutable {
        if (ec) {
          init.handler(ec, Packet());
          return;
        }

        // We can't directly pass init.handler to SignalResult::Wait
        // here, or the asio auto-coroutine magic gets confused and
        // thinks we are in a coroutine if SendReceivePacket was
        // called from a coroutine.
        SignalResult::Wait(
            service_, &read_signal_,
            parameters_.packet_timeout_s,
            [=](const boost::system::error_code& ec, Packet packet) mutable {
              init.handler(ec, packet);
            });
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

    ReadLoop1(boost::system::error_code());

    service_.post(std::bind(handler, boost::system::error_code()));
  }

  void ReadLoop1(boost::system::error_code ec) {
    if (ec) { throw boost::system::system_error(ec); }

    // Read until we get a frame header.
    boost::asio::async_read_until(
        *stream_, rx_streambuf_, "\xff\xff",
        std::bind(&HerkuleXProtocol::ReadLoop2, this,
                  std::placeholders::_1,
                  std::placeholders::_2));
  }

  void ReadLoop2(boost::system::error_code ec, std::size_t) {
    // Clear out the streambuf until we have our delimeter.
    std::istream istr(&rx_streambuf_);
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
    if (rx_streambuf_.size() < 5) {
      boost::asio::async_read(
          *stream_, rx_streambuf_,
          boost::asio::transfer_at_least(5 - rx_streambuf_.size()),
          std::bind(&HerkuleXProtocol::ReadLoop3, this,
                    std::placeholders::_1,
                    std::placeholders::_2));
    } else {
      ReadLoop3(boost::system::error_code(), 0);
    }
  }

  void ReadLoop3(boost::system::error_code ec, std::size_t) {
    if (ec) { throw boost::system::system_error(ec); }

    char header[5] = {};
    std::istream istr(&rx_streambuf_);
    istr.read(header, 5);
    if (istr.gcount() != 5) {
      throw std::runtime_error("inconsistent header");
    }

    const uint8_t size = header[0];

    if (size < 7) {
      // Malformed header.  Skip back to looking for
      // synchronization.  Probably should log?

      // TODO jpieper: Think about logging.
      ReadLoop1(boost::system::error_code());
      return;
    }

    if (rx_streambuf_.size() < (size - 7)) {
      boost::asio::async_read(
          *stream_,
          rx_streambuf_,
          boost::asio::transfer_at_least(
              (size - 7) - rx_streambuf_.size()),
          std::bind(&HerkuleXProtocol::ReadLoop4, this,
                    std::placeholders::_1,
                    std::placeholders::_2, header));
    } else {
      ReadLoop4(boost::system::error_code(), 0, header);
    }
  }

  void ReadLoop4(boost::system::error_code ec, std::size_t,
                 char header[5]) {
    if (ec) { throw boost::system::system_error(ec); }

    const uint8_t size = header[0];
    const uint8_t servo = header[1];
    const uint8_t command = header[2];
    const uint8_t cksum1 = header[3];
    const uint8_t cksum2 = header[4];

    std::istream istr(&rx_streambuf_);
    char data_buffer[256] = {};
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

    ReadLoop1(boost::system::error_code());
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
  char buffer_[256] = {};
  boost::asio::streambuf rx_streambuf_{512};
};

struct HerkuleXConstants {
  struct Register {
    constexpr Register(const uint8_t position, const uint8_t length)
        : position(position), length(length) {}

    uint8_t position;
    uint8_t length;
  };

  HerkuleXConstants();

  const std::map<std::string, Register> ram_registers;

  // RAM registers
  static constexpr Register temperature_c() { return Register{55, 1}; }
  static constexpr Register voltage() { return Register{54, 1}; }
  static constexpr Register position() { return Register{60, 2}; }
  // NOTE: Older versions of the datasheet say this should be a RAM
  // register 62, but the newer ones have the correct value of 64.
  static constexpr Register pwm() { return Register{64, 2}; }
  static constexpr Register cal_diff() { return Register{53, 1}; }
  static constexpr Register torque_control() { return Register{52, 1}; }


  // EEPROM registers
  static constexpr Register id() { return Register{6, 1}; }
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
    std::string register_data{Base::StatusResponse::data.substr(2, length)};
  };

  MemReadResponse MemRead(uint8_t command, uint8_t servo,
                          uint8_t reg, uint8_t length,
                          boost::asio::yield_context yield) {
    typename Base::Packet to_send;
    to_send.servo = servo;
    to_send.command = command;
    uint8_t buf[2] = { reg, length };
    to_send.data = std::string(reinterpret_cast<const char*>(buf), 2);

    typename Base::Packet result = this->SendReceivePacket(to_send, yield);
    if (result.servo != to_send.servo && to_send.servo != BROADCAST) {
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

  typename Base::StatusResponse Status(uint8_t servo,
                                       boost::asio::yield_context yield) {
    typename Base::Packet to_send;
    to_send.servo = servo;
    to_send.command = STAT;

    auto result = this->SendReceivePacket(to_send, yield);
    if (result.servo != to_send.servo && to_send.servo != BROADCAST) {
      throw std::runtime_error(
          (boost::format("Synchronization error, send status to servo 0x%02x, "
                         "response from 0x%02x") %
           static_cast<int>(to_send.servo) %
           static_cast<int>(result.servo)).str());
    }

    if (result.command != ACK_STAT) {
      throw std::runtime_error(
          (boost::format("Expected response command 0x%02x, received 0x%02x") %
           static_cast<int>(ACK_STAT) %
           static_cast<int>(result.command)).str());
    }

    if (result.data.size() != 2) {
      throw std::runtime_error(
          (boost::format("Received status of incorrect size, expected 2, "
                         "got %d") %
           result.data.size()).str());
    }

    return typename Base::StatusResponse(result);
  }

  void Reboot(uint8_t servo, boost::asio::yield_context yield) {
    typename Base::Packet to_send;
    to_send.servo = servo;
    to_send.command = REBOOT;

    this->SendPacket(to_send, yield);
  }

  void MemWrite(uint8_t command, uint8_t servo, uint8_t address,
                const std::string& data,
                boost::asio::yield_context yield) {
    typename Base::Packet to_send;
    to_send.servo = servo;
    to_send.command = command;
    std::ostringstream ostr;

    ostr.write(reinterpret_cast<const char*>(&address), 1);
    uint8_t length = data.size();
    ostr.write(reinterpret_cast<const char*>(&length), 1);

    ostr.write(data.data(), data.size());

    to_send.data = ostr.str();

    this->SendPacket(to_send, yield);
  }

  template <typename T>
  void MemWriteValue(uint8_t command, uint8_t servo, T field, int value_in,
                     boost::asio::yield_context yield) {
    int value = value_in;

    std::ostringstream ostr;
    for (int i = 0; i < field.length; i++) {
      uint8_t byte = value & 0xff;
      ostr.write(reinterpret_cast<const char*>(&byte), 1);
      value = value >> 8;
    }

    MemWrite(command, servo, field.position, ostr.str(), yield);
  }

  template <typename T>
  void RamWrite(uint8_t servo, T field, int value,
                boost::asio::yield_context yield) {
    MemWriteValue(RAM_WRITE, servo, field, value, yield);
  }

  template <typename T>
  void EepWrite(uint8_t servo, T field, int value,
                boost::asio::yield_context yield) {
    MemWriteValue(EEP_WRITE, servo, field, value, yield);
  }

  template <typename T>
  int MemReadValue(uint8_t command, uint8_t servo, T field,
                   boost::asio::yield_context yield) {
    auto response = MemRead(
        command, servo, field.position, field.length, yield);
    int result = 0;
    BOOST_ASSERT(response.register_data.size() >= field.length);
    for (int i = 0; i < field.length; i++) {
      result |= (static_cast<int>(response.register_data[i]) << (i * 8));
    }

    if (result >= (0x80 << ((field.length - 1) * 8))) {
      result = result - (1 << (field.length * 8));
    }

    return result;
  }

  template <typename T>
  int RamRead(uint8_t servo, T field,
              boost::asio::yield_context yield) {
    return MemReadValue(RAM_READ, servo, field, yield);
  }

  template <typename T>
  int EepRead(uint8_t servo, T field,
              boost::asio::yield_context yield) {
    return MemReadValue(EEP_READ, servo, field, yield);
  }

  template <typename Targets>
  void SJog(const Targets& targets,
            double time_s,
            boost::asio::yield_context yield) {
    std::ostringstream data;

    uint8_t int_time = static_cast<uint8_t>(
        std::max(0.0, std::min(255.0, time_s * 1000.0 / 11.2)));
    data.write(reinterpret_cast<const char*>(&int_time), 1);

    for (const auto& target: targets) {
      uint8_t buf[] = {
        target.address,
        static_cast<uint8_t>(target.position & 0xff),
        static_cast<uint8_t>((target.position >> 8) & 0xff),
        target.leds,
      };
      data.write(reinterpret_cast<const char*>(buf), sizeof(buf));
    }

    typename Base::Packet to_send;
    to_send.servo = BROADCAST;
    to_send.data = data.str();
    to_send.command = S_JOG;

    this->SendPacket(to_send, yield);
  }
};
}
