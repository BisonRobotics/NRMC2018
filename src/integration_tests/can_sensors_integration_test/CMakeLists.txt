cmake_minimum_required(VERSION 2.8.3)
project(can_sensors_integration_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  localization
)

catkin_package(
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  ${localization_INCLUDE_DIRS}
)

add_executable(can_sensors_integration_test src/can_sensors_integration_test.cpp)

add_dependencies(can_sensors_integration_test localization)

target_link_libraries(can_sensors_integration_test
  ${catkin_LIBRARIES}
  can_sensors_lib
)
