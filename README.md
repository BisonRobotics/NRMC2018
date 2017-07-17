# NRMC2018
Repository for the Bison Robotics NASA Robotic Mining Competition 2018 Entry

# Getting Started
## Environment
The default and recommended environment for this year
- Ubuntu 16.04 
  - Available at: https://www.ubuntu.com/download/desktop
- ROS Kinetic
  - Available at : http://wiki.ros.org/kinetic/Installation/Ubuntu
    - Follow instructions for desktop install (not full-desktop install)
- VREP
  - Download modified VREP from [here](https://github.com/BisonRobotics/VREP-ROS-Support/blob/master/vrep.tar.gz)
  - Unzip and move to /opt
  - Add an alias to your bashrc for easy launching : `echo "alias vrep='/opt/VREP/vrep.sh'" >> ~/.bashrc`

## Build tool
Using catkin-tools. See the following link for additional usage and info: https://catkin-tools.readthedocs.io/en/latest/
```
# Add repo and keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

# Install build tool
sudo apt-get update
sudo apt-get install python-catkin-tools
```

## Workspace
Our workspace is the NRM2018 repository this README lives in. Assuming you have your environment properly configured
run the following commands.
```
# Clone the repo to your computer
# If you have ssh keys with github setup on your pc
git clone git@github.com:BisonRobotics/NRM2018.git

# If not
git clone https://github.com/BisonRobotics/NRM2018.git

# Install workspace dependencies, respond y to all prompts
rosdep install --from-paths . --ignore-src --rosdistro=kinetic

# Build workspace
catkin build
```

## Connecting to and configuring the robot (NUC)
See [Network Config](https://github.com/BisonRobotics/NRM2018/wiki/Network-Configuration)

