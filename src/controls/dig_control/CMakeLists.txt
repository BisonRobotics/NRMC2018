cmake_minimum_required(VERSION 3.0.2)
project(dig_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  genmsg 
  actionlib_msgs 
  actionlib
  std_msgs
  message_generation
  geometry_msgs
  visualization_msgs
)

find_package (vesc_access REQUIRED)
find_package (new_sim REQUIRED)
find_package (imperio REQUIRED)


link_directories (
/usr/src/gmock/build
)

## Generate actions in the 'action' folder
add_action_files(
   DIRECTORY action
   FILES Outrigger.action Dig.action Dump.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp actionlib actionlib_msgs message_runtime message_generation rospy std_msgs
    #DEPENDS EIGEN3
)

include_directories(include ${catkin_INCLUDE_DIRS} )
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${vesc_access_INCLUDE_DIRS})
include_directories(${new_sim_INCLUDE_DIRS})
## Declare a C++ library

add_library(backhoe_controller_lib src/backhoe_controller/backhoe_controller.cpp)
add_library(outrigger_controller_lib src/outrigger_controller/outrigger_controller.cpp)
add_library(bucket_controller_lib src/bucket_controller/bucket_controller.cpp)
add_library(dig_dump_action_lib src/dig_dump_action/dig_dump_action.cpp)

add_executable(hoe_master src/hoe_master.cpp)

add_dependencies(hoe_master
  backhoe_controller_lib
  outrigger_controller_lib
  bucket_controller_lib
  sim_robot
  dig_dump_action_lib
)

target_link_libraries(dig_dump_action_lib backhoe_controller_lib bucket_controller_lib ${catkin_LIBRARIES} ${new_sim_LIBRARIES} ${vesc_access_LIBRARIES})


target_link_libraries(hoe_master
  ${new_sim_LIBRARIES} 
  ${catkin_LIBRARIES}
  backhoe_controller_lib
  outrigger_controller_lib
  bucket_controller_lib
  dig_dump_action_lib
  ${vesc_access_LIBRARIES}
)

add_executable(outrigger_master src/outrigger_master.cpp)
target_link_libraries(outrigger_master ${catkin_LIBRARIES} outrigger_controller_lib sim_robot)

add_executable(outrigger_tester src/test/outrigger_tester.cpp)
target_link_libraries(outrigger_tester ${catkin_LIBRARIES})

add_dependencies (outrigger_tester ${PROJECT_NAME}_gencpp)
add_dependencies (outrigger_master ${PROJECT_NAME}_gencpp)

