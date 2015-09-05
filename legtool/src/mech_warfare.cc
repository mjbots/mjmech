// Copyright 2014-2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "mech_warfare.h"

#include <boost/property_tree/json_parser.hpp>

#include "property_tree_archive.h"

namespace legtool {

namespace {
struct EnableArchive {
  EnableArchive(std::map<std::string, bool>& enabled): enabled(enabled) {}

  template <typename NameValuePair>
  void Visit(const NameValuePair& pair) {
    Helper(pair.name(), pair.value(), 0);
  }

  template <typename T>
  auto Helper(const char* name, T* value, int)
      -> decltype((*value)->AsyncStart(ErrorHandler())) {
    enabled[name] = true;
  }

  template <typename T>
  void Helper(const char* name, T* value, long) {}

  std::map<std::string, bool>& enabled;
};
}

MechWarfare::MechWarfare(boost::asio::io_service& service)
    : service_(service),
      factory_(service),
      server_(service) {

  m_.servo_base.reset(new ServoBase(service_, factory_));
  m_.servo.reset(new Servo(m_.servo_base.get()));

  EnableArchive enable_archive(parameters_.enabled);
  m_.Serialize(&enable_archive);
}

namespace {
struct StartArchive {
  StartArchive(std::map<std::string, bool>& enabled, ErrorHandler handler)
      : enabled(enabled),
        joiner(std::make_shared<ErrorHandlerJoiner>(handler)) {}

  template <typename NameValuePair>
  void Visit(const NameValuePair& pair) {
    Helper(pair.name(), pair.value(), 0);
  }

  template <typename T>
  auto Helper(const char* name, T* value, int)
      -> decltype((*value)->AsyncStart(ErrorHandler())) {
    if (enabled[name]) {
      (*value)->AsyncStart(
          joiner->Wrap(std::string("starting: '") + name + "'"));
    }
  }

  template <typename T>
  void Helper(const char* name, T* value, long) {}

  std::map<std::string, bool>& enabled;
  std::shared_ptr<ErrorHandlerJoiner> joiner;
};
}

void MechWarfare::AsyncStart(ErrorHandler handler) {
  try {
    RippleConfig ripple_config;
    ripple_config = LoadRippleConfig();

    m_.gait_driver->SetGait(std::unique_ptr<RippleGait>(
                                new RippleGait(ripple_config)));

    NetworkListen();
  } catch (SystemError& se) {
    service_.post(std::bind(handler, se.error_code()));
    return;
  }

  StartArchive archive(parameters_.enabled, handler);
  m_.Serialize(&archive);
}

RippleConfig MechWarfare::LoadRippleConfig() {
  std::ifstream inf(parameters_.gait_config);
  if (!inf.is_open()) {
    throw SystemError::syserrno(
        "while opening gait config: '" + parameters_.gait_config + "'");
  }

  boost::property_tree::ptree tree;
  boost::property_tree::read_json(inf, tree);
  RippleConfig ripple_config;
  PropertyTreeReadArchive(tree).Accept(&ripple_config);

  // TODO jpieper: Load each leg's IK settings.

  return ripple_config;
}

void MechWarfare::NetworkListen() {
  boost::asio::ip::udp::endpoint endpoint(
      boost::asio::ip::udp::v4(), parameters_.port);
  server_.open(boost::asio::ip::udp::v4());
  server_.bind(endpoint);
  StartRead();
}

void MechWarfare::StartRead() {
  server_.async_receive_from(
      boost::asio::buffer(receive_buffer_),
      receive_endpoint_,
      std::bind(&MechWarfare::HandleRead, this,
                std::placeholders::_1,
                std::placeholders::_2));
}

void MechWarfare::HandleRead(ErrorCode ec, std::size_t size) {
  FailIf(ec);

  std::string data(receive_buffer_, size);
  StartRead();

  boost::property_tree::ptree tree;
  try {
    std::istringstream inf(data);
    boost::property_tree::read_json(inf, tree);
  } catch (std::exception& e) {
    std::cerr << "error reading network command: " << e.what() << "\n";
    return;
  }

  HandleMessage(tree);
}

void MechWarfare::HandleMessage(const boost::property_tree::ptree& tree) {
  auto optional_gait = tree.get_child_optional("gait");
  if (optional_gait) { HandleMessageGait(*optional_gait); }
}

void MechWarfare::HandleMessageGait(const boost::property_tree::ptree& tree) {
  Command command;
  PropertyTreeReadArchive(tree).Accept(&command);
  m_.gait_driver->SetCommand(command);
}

}
