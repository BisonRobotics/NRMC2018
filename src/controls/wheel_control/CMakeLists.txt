cmake_minimum_required(VERSION 2.8.3)
project(wheel_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  genmsg 
  actionlib_msgs 
  actionlib
  std_msgs
  vesc_access 
)



link_directories (
/usr/src/gmock/build
)


add_action_files(DIRECTORY action FILES distance.action)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES Wheels velocity_plugins
    CATKIN_DEPENDS roscpp pluginlib
    #DEPENDS EIGEN3
)

include_directories (include ${catkin_INCLUDE_DIRS} )
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${vesc_access_INCLUDE_DIRS})
## Declare a C++ library

add_library(velocity_plugins
  src/DifferentialDriveController.cpp
  src/Wheels.cpp
)

add_executable (wheel_goal_master src/wheel_goal_master.cpp)
target_link_libraries (wheel_goal_master ${catkin_LIBRARIES})
target_link_libraries(velocity_plugins ${catkin_LIBRARIES})

add_library(Wheels src/Wheels.cpp)

add_library(pos_control src/position_controller/position_controller.cpp)

## Tests
catkin_add_gtest(test_Wheels src/tests/test_Wheels.cpp src/Wheels.cpp)
if(TARGET test_Wheels)
  target_link_libraries(test_Wheels
    ${catkin_LIBRARIES}
  )
endif()

catkin_add_gtest(test_Interface src/tests/test_Interface.cpp include/wheel_control/Interface.h src/Wheels.cpp)
if(TARGET test_Interface)
  target_link_libraries(test_Interface
    ${catkin_LIBRARIES}
  )
endif()

catkin_add_gtest(test_velocity_plugins src/tests/test_velocity_plugins.cpp)
if(TARGET test_velocity_plugins)
  target_link_libraries(test_velocity_plugins
    ${catkin_LIBRARIES}
    Wheels
  )
  add_dependencies(test_velocity_plugins velocity_plugins)
endif()

catkin_add_gtest(test_position_controller src/position_controller/test/position_controller_tests.cpp)
if(TARGET test_position_controller)
  target_link_libraries(
    test_position_controller
    ${catkin_LIBRARIES}
    pos_control
  )
  add_dependencies(test_position_controller vesc_access)
endif()


