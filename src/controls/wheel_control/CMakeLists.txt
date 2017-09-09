cmake_minimum_required(VERSION 2.8.3)
project(wheel_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  
  )



link_directories (
/usr/src/gmock/build
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES Wheels velocity_plugins
    CATKIN_DEPENDS roscpp pluginlib
    #DEPENDS EIGEN3
)

include_directories (include ${catkin_INCLUDE_DIRS} )
include_directories(${GTEST_INCLUDE_DIRS})
## Declare a C++ library

add_library(vesc_lib
  src/vesc_socket_can/vesc_socket_can.cpp
)
add_library (vesc_access
  src/vesc_access/vesc_access.cpp
)
 
## Specify libraries to link a library or executable target against
target_link_libraries(vesc_access
  ${catkin_LIBRARIES}
  vesc_lib
)

add_library(velocity_plugins
  src/DifferentialDriveController.cpp
  src/Wheels.cpp
)

target_link_libraries(velocity_plugins ${catkin_LIBRARIES})

add_library(Wheels src/Wheels.cpp)


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

catkin_add_gtest(test_vesc_access src/vesc_access/tests/vesc_access_tests.cpp)
if (TARGET test_vesc_access)
  target_link_libraries(
    test_vesc_access
    ${catkin_LIBRARIES}
    vesc_access
    gmock_main
  )
endif ()
