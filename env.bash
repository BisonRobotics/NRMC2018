#!/usr/bin/env bash
source /opt/ros/kinetic/setup.bash
source /home/$(whoami)/NRMC2018/devel/setup.bash
export ROS_MASTER_URI=http://her-name:1234
exec "$@"
