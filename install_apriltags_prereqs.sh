#!/bin/bash

cd ~
git clone git@github.com:xenobot-dev/apriltags_ros.git
sudo apt-get install libcairo2-dev libcgal-dev libcgal-qt5-dev
cd apriltags_ros/software/apriltags-cpp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
sudo make install
 
