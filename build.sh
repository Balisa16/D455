#! /bin/bash

git submodule update --init --recursive

build_path="build"

if [ ! -d "$build_path" ]; then
	echo "Create build folder"
    mkdir build
fi
cd build
cmake .. && make -j4
cd ..