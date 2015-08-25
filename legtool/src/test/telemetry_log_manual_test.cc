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

#include "telemetry_log.h"

#include <boost/program_options.hpp>

namespace {
using namespace legtool;

int work(int argc, char** argv) {
  namespace po = boost::program_options;

  po::options_description desc("Allowable options");

  std::string output;
  bool realtime = false;
  int count = 1;
  int delay_us = 10000;
  int size = 100;
  desc.add_options()
      ("output,o", po::value(&output), "output file")
      ("realtime,r", po::bool_switch(&realtime), "realtime mode")
      ("count,c", po::value(&count), "blocks to write")
      ("delay_us,d", po::value(&delay_us), "delay between blocks")
      ("size,s", po::value(&size), "size of blocks")
      ("help,h", "display usage message")
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cerr << desc;
    return 0;
  }

  TelemetryLog log;
  log.SetRealtime(realtime);
  log.Open(output);

  for (int i = 0; i < count; i++) {
    log.WriteBlock(TelemetryFormat::BlockType::kBlockData,
                   std::string(size, ' '));
    ::usleep(delay_us);
  }

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
