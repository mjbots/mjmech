#!/bin/sh

set -ev

./install-packages --yes

./tools/bazel test //...
./tools/bazel build -c opt --config=pi //mech:deploy.tar
