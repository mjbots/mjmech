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

#include "herkulex.h"

#include <boost/test/auto_unit_test.hpp>

#include "comm_factory.h"

using namespace legtool;

BOOST_AUTO_TEST_CASE(HerkuleXTest) {
  boost::asio::io_service service;
  typedef StreamFactory<PipeGenerator> Factory;
  Factory factory(service);
  typedef HerkuleX<Factory> Servo;
  Servo herkulex(service, factory);

  herkulex.parameters()->stream.type = "pipe";
  herkulex.parameters()->stream.Get<PipeGenerator>()->key = "test";

  auto stream = factory.generator<PipeGenerator>()->GetStream(
      service, "test", PipeGenerator::Mode::kDirectionB);

  boost::asio::streambuf streambuf;
  boost::asio::spawn(service, [&](boost::asio::yield_context yield) {
      char buf[4096] = {};
      while (true) {
        std::size_t bytes_read =
            stream->async_read_some(boost::asio::buffer(buf), yield);
        std::ostream ostr(&streambuf);
        ostr.write(buf, bytes_read);
      }
    });

  boost::asio::spawn(service, [&](boost::asio::yield_context yield) {
      herkulex.AsyncStart(yield);

      Servo::Packet packet;
      packet.servo = 0xfd;
      packet.command = Servo::STAT;
      packet.data = "";
      herkulex.SendPacket(packet, yield);

      // The packet should have ended up in our streambuf now.
      {
        std::ostringstream ostr;
        ostr << &streambuf;
        std::string data = ostr.str();
        BOOST_CHECK_EQUAL(data.size(), 7);
        BOOST_CHECK_EQUAL(data, "\xff\xff\x07\xfd\x07\xfc\x02");
      }

      // Start listening for a packet.
      boost::optional<Servo::Packet> listen;
      boost::asio::spawn(service, [&](boost::asio::yield_context yield) {
          listen = herkulex.ReceivePacket(yield);
        });

      BOOST_CHECK(!listen);

      // Send out a response over the wire.
      std::string response("\xff\xff\x09\xfd\x47\xf2\x0c\x01\x40");
      boost::asio::async_write(*stream,  boost::asio::buffer(response), yield);

      // At this point, the other coroutine should have finished with
      // a receipt.
      BOOST_REQUIRE(listen);
      BOOST_CHECK_EQUAL(listen->data, "\x01\x40");
      BOOST_CHECK_EQUAL(listen->servo, 0xfd);
      BOOST_CHECK_EQUAL(listen->command, Servo::ACK_STAT);
      BOOST_CHECK_EQUAL(listen->cksum_good, true);

      Servo::StatusResponse status(*listen);
      BOOST_CHECK_EQUAL(status.reg48, 0x01);
      BOOST_CHECK_EQUAL(status.reg49, 0x40);
      BOOST_CHECK_EQUAL(status.exceeded_input_voltage_limit, true);
      BOOST_CHECK_EQUAL(status.exceeded_allowed_pot_limit, false);
    });

  service.run();
}
