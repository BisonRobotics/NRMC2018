cmake_minimum_required(VERSION 2.8.3)
project(new_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  vesc_access
  sensor_access
  localization
)

find_package (vesc_access REQUIRED)
find_package (sensor_access REQUIRED)
find_package (localization REQUIRED)

link_directories (
/usr/src/gmock/build
)

catkin_package(
  INCLUDE_DIRS include/ 
  LIBRARIES sim_robot
  DEPENDS vesc_access sensor_access localization
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${vesc_access_INCLUDE_DIRS}
  ${sensor_access_INCLUDE_DIRS}
  include/
  ${localization_INCLUDE_DIRS}
)

add_library(sim_robot
  src/sim_robot.cpp
)

#include_directories(${GTEST_INCLUDE_DIRS})