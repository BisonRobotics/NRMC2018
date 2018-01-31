# NRMC2018
Repository for the Bison Robotics NASA Robotic Mining Competition 2018 Entry

# Getting Started
## Environment
The default and recommended environment for this year
- Ubuntu 16.04 
  - Available at: https://www.ubuntu.com/download/desktop
- ROS Kinetic
  - Available at : http://wiki.ros.org/kinetic/Installation/Ubuntu
    - Follow instructions for desktop install (not full-desktop install)`

## Configure git and other helpful utils
```
sudo apt-get update
sudo apt-get install git tmux ssh vim
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash 
sudo apt-get install -y git-lfs
```

## Setup your ssh keys
Follow these instructions https://help.github.com/articles/connecting-to-github-with-ssh/

## Setup an IDE
I highly recommend using the clion IDE available at https://www.jetbrains.com/clion/. Make sure you sign up for an education account so you don't have to pay for it. Otherwise if you're as hardcore as Dr. Ding, VIM works too.

## Install and build gmock
```
sudo apt-get install build-essential google-mock
cd /usr/src/gmock
sudo mkdir build
cd build
sudo cmake ../
sudo make
```

## Install clang
```
sudo apt-get install clang-format-3.6
```

## Workspace
Our workspace is the NRM2018 repository this README lives in. Assuming you have your environment properly configured
run the following commands.

```
# Clone the repo to your computer
cd ~

# If you have ssh keys with github setup on your pc
git clone git@github.com:BisonRobotics/NRMC2018.git

# If not
git clone https://github.com/BisonRobotics/NRMC2018.git
````
From here make sure you checkout the branch you'll be working on before proceeding
```
# Checkout branch
git checkout <branch_name>

# Pull in submodules
git submodule update --init --recursive

# Install workspace dependencies, respond y to all prompts
rosdep install --from-paths . --ignore-src --rosdistro=kinetic --default-yes -r --os=ubuntu:xenial

# Build workspace
catkin_make
```
If you are using the VREP simulator
```
# Link vrep plugin to vrep install directory (Assumes you placed NRMC2018 and vrep in home directory)
ln -s ~/NRMC2018/devel/lib/libv_repExtRosInterface.so ~/vrep/
```
Add some optional alias that make launching stuff and working with ROS easier
```
echo "alias vrep='~/vrep/vrep.sh'"                    >> ~/.bashrc
echo "alias clion='~/clion/bin/clion.sh'"             >> ~/.bashrc
echo "alias ws='cd ~/NRMC2018'"                       >> ~/.bashrc
echo "alias wss='source ~/NRMC2018/devel/setup.bash'" >> ~/.bashrc
```
To start the simulator
```
# Make sure you run each command in a new terminal and have your workspace sourced
roscore
vrep
roslaunch description display.launch
```
## Useful unit test commands for debugging
# Build and run single test suite
Tab complete is your friend here

catkin_make run_tests_wheel_control_gtest_test_WaypointControllerHelper2
```
# Build test for debugging 
catkin_make -DCMAKE_BUILD_TYPE=Debug test_WaypointControllerHelper2
```
# Debug the unit test
gdb ./devel/lib/wheel_control/test_WaypointControllerHelper2
```


## Known Issues/Bugs in "Not-Our-Stuff"
# CANables
You may not be able to enumerate two CANables on the same computer easily and as you would imagine. There is a bug in the CANable firmware which makes one CANable enumerate as two. So, to work around this issue: make one of them can1 and the other can2. Forget about can0. Less than nothing, it decieved you, and so should be scorned.

sudo ip link set can1 type can bitrate 500000 triple-sampling on
sudo ip link set can1 up

sudo ip link set can2 type can bitrate 500000 triple-sampling on
sudo ip link set can2 up

