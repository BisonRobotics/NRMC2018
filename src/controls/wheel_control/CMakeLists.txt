cmake_minimum_required(VERSION 2.8.3)
project(wheel_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  nav_msgs
  sensor_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    #INCLUDE_DIRS include
    #LIBRARIES BasicSkidSteer
    #CATKIN_DEPENDS nav_msgs roscpp sensor_msgs
    #DEPENDS EIGEN3
)

###########
## Build ##
###########

include_directories (include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
add_library(velocity_plugins
  src/DifferentialDriveController.cpp
  src/Wheels.cpp
)


## Add gtest based cpp test target and link libraries
catkin_add_gtest(test_Wheels src/tests/test_Wheels.cpp src/Wheels.cpp)
if(TARGET test_Wheels)
  target_link_libraries(test_Wheels
    ${catkin_LIBRARIES}
  )
endif()

catkin_add_gtest(test_Interface src/tests/test_Interface.cpp include/Interface.h src/Wheels.cpp)
if(TARGET test_Interface)
  target_link_libraries(test_Interface
    ${catkin_LIBRARIES}
  )
endif()

catkin_add_gtest(test_velocity_plugins src/tests/test_velocity_plugins.cpp src/Wheels.cpp)
if(TARGET test_velocity_plugins)
  target_link_libraries(test_velocity_plugins
    ${catkin_LIBRARIES}
  )
  add_dependencies(test_velocity_plugins velocity_plugins)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
