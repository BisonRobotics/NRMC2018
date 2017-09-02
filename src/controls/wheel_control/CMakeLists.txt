cmake_minimum_required(VERSION 2.8.3)
project(wheel_control)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES Wheels velocity_plugins
    CATKIN_DEPENDS roscpp pluginlib
    #DEPENDS EIGEN3
)

include_directories (include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
add_library(vesc_lib
  src/VescSocketCAN.cpp
)
add_library (robot_parameter_wrapper
  src/RobotParameterWrapper/RobotParameterWrapper.cpp
)
 
## Specify libraries to link a library or executable target against
target_link_libraries(robot_parameter_wrapper
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


