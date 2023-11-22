#!/bin/sh
set -e

pushd flutter_bullet_library
make
popd
dart run ffigen --config ffigen.yaml
