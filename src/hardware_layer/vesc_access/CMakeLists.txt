cmake_minimum_required(VERSION 2.8.3)
project(vesc_access)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
        sensor_msgs
        std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

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
  INCLUDE_DIRS include
  LIBRARIES vesc_access safety_vesc
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories (
${GTEST_INCLUDE_DIRS}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(vesc_lib
        src/vesc_control/vesc_socket_can.cpp
)

add_library (vesc_access
  src/vesc_access/vesc_access.cpp
  src/vesc_control/vesc_socket_can.cpp
)

add_library(safety_vesc src/safety_vesc/safety_controller.cpp src/safety_vesc/linear_safety_controller.cpp src/safety_vesc/backhoe_safety_controller.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(vesc_access
  ${catkin_LIBRARIES}
  #vesc_lib
)

target_link_libraries(vesc_lib
        ${catkin_LIBRARIES})
add_executable (vesc_access_integration_test
  test/vesc_access_integration_test.cpp
)

add_executable (vesc_socket_can_integration_test
  test/vesc_socket_can_integration_test.cpp
)

target_link_libraries (vesc_socket_can_integration_test
  vesc_lib
)
 
target_link_libraries (vesc_access_integration_test
  vesc_access
)


add_executable(vesc_access_pot_limit_test
        test/vesc_access_pot_limit_test.cpp
        )

target_link_libraries(vesc_access_pot_limit_test
        vesc_access)
add_executable(vesc_socket_pot_test
        test/vesc_socket_pot_test.cpp
        )

target_link_libraries(vesc_socket_pot_test vesc_lib ${catkin_LIBRARIES})

add_executable(runtime_check test/vesc_runtime_check.cpp)
target_link_libraries(runtime_check vesc_lib)


add_executable(linear_test test/linear_integration_test.cpp )
target_link_libraries(linear_test ${catkin_LIBRARIES} safety_vesc vesc_access)
# Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/vesc_access.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/vesc_access_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 install(TARGETS  vesc_access
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(TARGETS  safety_vesc
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/vesc_access/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
 )
 install(DIRECTORY include/safety_vesc/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vesc_access.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
catkin_add_gtest(test_vesc_access src/vesc_access/tests/vesc_access_tests.cpp)
if (TARGET test_vesc_access)
  target_link_libraries(
    test_vesc_access
    ${catkin_LIBRARIES}
    vesc_access
    gmock_main
  )
endif ()

catkin_add_gtest(test_safety_vesc src/safety_vesc/test/safety_controller_test.cpp)
if (TARGET test_safety_vesc)
  target_link_libraries(
          test_safety_vesc
          safety_vesc
          ${catkin_LIBRARIES}
          gmock_main
          gtest
  )
endif ()


catkin_add_gtest(test_backhoe_safety_vesc src/safety_vesc/test/backhoe_safety_controller_test.cpp)
if (TARGET test_backhoe_safety_vesc)
  target_link_libraries(
          test_backhoe_safety_vesc
          safety_vesc
          ${catkin_LIBRARIES}
          gmock_main
          gtest
  )
endif ()

catkin_add_gtest(test_linear_safety_vesc src/safety_vesc/test/linear_safety_controller_test.cpp)
if (TARGET test_linear_safety_vesc)
  target_link_libraries(
          test_linear_safety_vesc
          safety_vesc
          ${catkin_LIBRARIES}
          gmock_main
          gtest
  )
endif ()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
