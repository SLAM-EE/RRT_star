#!/usr/bin/env sh
mkdir spline/build && cd spline/build
cmake ..
make all
cd ../../
mkdir build && cd build
cmake ..
make all
