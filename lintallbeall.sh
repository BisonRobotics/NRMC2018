#!/bin/bash

find . -path ./src/lpms_imu -prune -o \
       -path ./src/realsense_samples_ros -prune -o \
       -path ./src/ros_vrep_plugin -prune -o \
       -path ./src/timesync -prune -o \
       -name '*.h' -or -name '*.cpp' | xargs clang-format-3.6 -i -style=file $1
