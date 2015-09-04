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

#include MODULE_HEADER_FILE

#include <boost/program_options.hpp>

#include "fail.h"
#include "program_options_archive.h"
#include "telemetry_log.h"
#include "telemetry_log_registrar.h"
#include "telemetry_registry.h"

using namespace legtool;

namespace {
struct Context {
  boost::asio::io_service service;
  TelemetryLog telemetry_log;
  TelemetryRegistry<TelemetryLogRegistrar> telemetry_registry{&telemetry_log};
};

struct FailWrapper {
  void operator()(ErrorCode ec) const {
    FailIf(ec);
  }
};

int safe_main(int argc, char**argv) {
  namespace po = boost::program_options;

  Context context;
  MODULE_CLASS_NAME module(context);

  std::string config_file;
  std::string log_file;

  po::options_description desc("Allowable options");
  desc.add_options()
      ("help,h", "display usage message")
      ("config,c", po::value(&config_file), "read options from file")
      ("log,l", po::value(&log_file), "write to log file")
      ;

  ProgramOptionsArchive(&desc).Accept(module.parameters());

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  if (!config_file.empty()) {
    po::store(po::parse_config_file<char>(config_file.c_str(), desc), vm);
  }
  po::notify(vm);

  if (vm.count("help")) {
    std::cerr << desc;
    return 0;
  }

  if (!log_file.empty()) {
    context.telemetry_log.Open(log_file);
  }

  module.AsyncStart(FailWrapper());

  context.service.run();
  return 0;
}
}

int main(int argc, char**argv) {
  try {
    return safe_main(argc, argv);
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  } catch (...) {
    std::cerr << "Unknown error.\n";
    return 2;
  }
}
