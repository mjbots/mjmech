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

#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "gait.h"
#include "program_options_archive.h"
#include "property_tree_archive.h"

namespace {
using namespace legtool;

int work(int argc, char** argv) {
  boost::asio::io_service service;

  namespace po = boost::program_options;

  std::string target = "192.168.0.123";
  Command command;

  po::options_description desc("Allowable options");
  desc.add_options()
      ("help,h", "display usage message")
      ("target,t", po::value(&target), "destination of commands")
      ;

  ProgramOptionsArchive(&desc, "cmd.").Accept(&command);

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cerr << desc;
    return 0;
  }

  std::string host;
  std::string port_str;
  const size_t colon = target.find_first_of(':');
  if (colon != std::string::npos) {
    host = target.substr(0, colon);
    port_str = target.substr(colon + 1);
  } else {
    host = target;
    port_str = "13356";
  }

  namespace pt = boost::property_tree;

  std::ostringstream ostr;
  pt::ptree tree;
  tree.add_child("gait", PropertyTreeWriteArchive().Accept(&command).tree());
  pt::write_json(ostr, tree);
  std::string message = ostr.str();

  typedef boost::asio::ip::udp udp;

  udp::resolver resolver(service);
  auto it = resolver.resolve(udp::resolver::query(host, port_str));

  udp::socket socket(service, udp::v4());
  socket.send_to(boost::asio::buffer(message), *it);

  return 0;
}
}

int main(int argc, char** argv) {
  try {
    return work(argc, argv);
  } catch (std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
