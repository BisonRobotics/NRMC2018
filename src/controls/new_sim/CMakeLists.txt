cmake_minimum_required(VERSION 2.8.3)
project(new_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package (vesc_access REQUIRED)
find_package (sensor_access REQUIRED)
find_package (localization REQUIRED)

link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

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

add_library(support_libs
  src/sim_imu.cpp
  src/sim_pos.cpp
  src/sim_vesc.cpp
  src/sim_backhoe.cpp
  src/sim_bucket.cpp
  src/sim_outriggers.cpp
)


target_link_libraries (
  support_libs
  ${vesc_access_LIBRARIES}
  ${sensor_access_LIBRARIES}
)

target_link_libraries (sim_robot ${vesc_access_LIBRARIES} ${sensor_access_LIBRARIES} support_libs)

#include_directories(${GTEST_INCLUDE_DIRS})

## Mark executables and/or libraries for installation
 install(TARGETS sim_robot
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/sim_robot
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )


