#!/bin/bash

pushd third_party/grpc
git submodule update --init --recursive --depth 1

mkdir -p cmake/build
pushd cmake/build
cmake -DgRPC_BUILD_TESTS=OFF \
      -DgRPC_INSTALL=ON \
      -DCMAKE_INSTALL_PREFIX="$(pwd)/../../install" \
      ../..
make -j 8
make install