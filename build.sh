#! /bin/bash

build_path="build"

if [ ! -d "$build_path" ]; then
	printf "\033[32mCreate build folder\033[0m\n"
    mkdir build
fi
cd build
make -j4
cd ..