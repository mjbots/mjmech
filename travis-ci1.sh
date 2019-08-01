#!/bin/sh

set -ev

sudo apt-get update
./install-packages --yes

./tools/bazel test //base/...
