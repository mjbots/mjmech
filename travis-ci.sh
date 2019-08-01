#!/bin/sh

set -ev

./tools/bazel test //...
./tools/bazel build -c opt --config=pi //mech:deploy.tar
