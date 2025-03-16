#!/bin/bash
mkdir repos
cd repos
git clone https://gitlab.com/libeigen/eigen.git
cd eigen
mkdir build && cd build && cmake .. && make -j4 && sudo make install
cd ../..
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build && cd build && cmake .. && make -j4 && sudo make install
cd ../..
git clone https://github.com/artivis/manif.git
cd manif
mkdir build && cd build && cmake .. && make -j4 && sudo make install
cd ../..
sudo apt-get install libssl-dev
sudo apt-get install libusb-dev
sudo apt-get install libglfw3-dev
wget "https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.53.1.zip"
unzip v2.53.1.zip
cd librealsense-2.53.1
mkdir build && cd build && cmake .. -DBUILD_EXAMPLES=OFF && make -j4 && sudo make install



