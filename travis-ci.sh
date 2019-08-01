#!/bin/sh

set -ev

sudo apt-get update
./install-packages --yes

./tools/bazel test //base/...

#./tools/bazel test //...
#./tools/bazel build -c opt --config=pi //mech:deploy.tar
