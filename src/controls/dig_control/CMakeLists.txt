cmake_minimum_required(VERSION 3.0.2)
project(dig_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  genmsg 
  actionlib_msgs 
  message_generation
  actionlib
  std_msgs
  vesc_access 
  #localization
  geometry_msgs
  visualization_msgs
  new_sim
  imperio
)

link_directories (
/usr/src/gmock/build
)

## Generate actions in the 'action' folder
add_action_files(
   DIRECTORY action
   FILES central_drive.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs   std_msgs
 )


catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp actionlib actionlib_msgs message_generation rospy std_msgs
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


add_executable(hoe_master src/hoe_master.cpp)

add_dependencies(hoe_master
  backhoe_controller_lib
  outrigger_controller_lib
  bucket_controller_lib
  sim_robot
)


target_link_libraries(hoe_master
  ${new_sim_LIBRARIES} 
  ${catkin_LIBRARIES}
  backhoe_controller_lib
  outrigger_controller_lib
  bucket_controller_lib
  ${vesc_access_LIBRARIES}
)



#add_executable(central_drive_server src/central_drive_server.cpp)

#target_link_libraries(
#  central_drive_server
#  ${catkin_LIBRARIES}
#)

#add_dependencies(
#  central_drive_server
#  ${backhoe_control_EXPORTED_TARGETS}
#)

#add_executable(central_drive_client src/central_drive_client.cpp)

#target_link_libraries(
#  central_drive_client
#  ${catkin_LIBRARIES}
#)

#add_dependencies(
#  central_drive_client
#  ${backhoe_control_EXPORTED_TARGETS}
#)


## Tests

#catkin_add_gtest(test_position_controller src/position_controller/test/position_controller_tests.cpp)
#if(TARGET test_position_controller)
#  target_link_libraries(
#    test_position_controller
#    ${catkin_LIBRARIES}
#    pos_control
#    gmock_main
#  )
#  add_dependencies(test_position_controller vesc_access)
#endif()


