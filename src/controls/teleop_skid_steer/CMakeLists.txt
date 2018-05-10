cmake_minimum_required(VERSION 2.8.3)
project(teleop_skid_steer)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  vesc_access

)


link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES teleop_interface
#  CATKIN_DEPENDS joy rospy sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories (include ${catkin_INCLUDE_DIRS})
include_directories (${GTEST_INCLUDE_DIRS})
include_directories (${vesc_access_INCLUDE_DIRS})



add_library( teleop_interface
  src/teleop_interface/teleop_interface.cpp
)

target_link_libraries (teleop_interface ${catkin_LIBRARIES} vesc_access)

add_executable (teleop_interface_exec src/teleop.cpp)

target_link_libraries (teleop_interface_exec teleop_interface)


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
 install(TARGETS teleop_interface
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/teleop_interface
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
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
#catkin_add_gtest(test_teleop src/teleop_interface/tests/teleop_interface_tests.cpp)
#if(TARGET test_teleop)
#   target_link_libraries(test_teleop ${catkin_LIBRARIES} teleop_interface gmock_main)
#   add_dependencies (test_teleop vesc_access)
#endif()


