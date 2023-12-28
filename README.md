# D455

Sampling code for D455 Camera Sampling System

## Dependencies
```
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir build && cd build
cmake .. && make -j4
sudo make install
```

## Build
```
git clone https://github.com/Balisa16/D455.git
cd D455
git submodule update --init --recursive
./prebuild.sh
mkdir build && cd build
cmake .. && make -j$(( $(nproc) - 1 ))
```