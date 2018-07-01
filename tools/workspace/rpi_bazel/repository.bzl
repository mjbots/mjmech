# -*- python -*-

# Copyright 2018 Josh Pieper, jjp@pobox.com.
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

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def rpi_bazel_repository(name):
    commit = "32aeb600bd082e8f80613bfcbe38f3b4222b02ff"
    http_archive(
        name = name,
        url = "https://github.com/mjbots/rpi_bazel/archive/{}.zip".format(commit),
        sha256 = "37c81f86ed38d53a0dfa51ecf7f7f38a4c25d682ffc05271704e5dc9fa6691d3",
        strip_prefix = "rpi_bazel-{}".format(commit),
    )