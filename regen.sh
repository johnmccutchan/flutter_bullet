#!/bin/sh
set -e

pushd native
make
popd
dart run ffigen --config ffigen.yaml
