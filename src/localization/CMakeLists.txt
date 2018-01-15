cmake_minimum_required(VERSION 2.8.3)
project(localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
)

find_package (catkin REQUIRED cmake_modules)
find_package (Eigen3 REQUIRED)
find_package (vesc_access REQUIRED)
find_package (sensor_access REQUIRED)

link_directories (
/usr/src/gmock/build
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
  INCLUDE_DIRS include/ 
  LIBRARIES localizer_lib super_localizer_lib
#  CATKIN_DEPENDS nav_msgs roscpp
   DEPENDS EIGEN3 vesc_access sensor_access
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${vesc_access_INCLUDE_DIRS}
  ${sensor_access_INCLUDE_DIRS}
  include/
)



include_directories(${GTEST_INCLUDE_DIRS})
include_directories (${EIGEN3_INCLUDE_DIRS}) #figure that one out

add_library(localizer_lib
	src/localizer/localizer.cpp
)


add_library(super_localizer_helper_lib
    src/super_localizer/super_localizer_helper.cpp
)

add_library(super_localizer_lib
	src/super_localizer/super_localizer.cpp
		)
add_dependencies(localizer_lib vesc_access lpresearch_imu )

target_link_libraries(localizer_lib
 vesc_access
 lpresearch_imu
)



add_dependencies(super_localizer_lib localizer_lib)

add_dependencies(super_localizer_helper_lib localizer_lib)


target_link_libraries(super_localizer_helper_lib 
 localizer_lib
)

target_link_libraries(super_localizer_lib
 localizer_lib
 super_localizer_helper_lib

)

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
 install(TARGETS localizer_lib
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_localization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

catkin_add_gtest(test_localization src/localizer/tests/localizer_tests.cpp)
if(TARGET test_localization)
  target_link_libraries(
    test_localization
    ${catkin_LIBRARIES}
    localizer_lib
    gmock_main
  )
  add_dependencies(test_localization vesc_access localizer_lib)
endif()

catkin_add_gtest(test_super_localization src/super_localizer/tests/super_localizer_tests.cpp)
if(TARGET test_super_localization)
  target_link_libraries(
    test_super_localization
    ${catkin_LIBRARIES}
    super_localizer_lib
    gmock_main
  )
  add_dependencies(test_super_localization vesc_access super_localizer_lib)
endif()

catkin_add_gtest(test_super_localization_helper src/super_localizer/tests/super_localizer_helper_tests.cpp)
if(TARGET test_super_localization_helper)
  target_link_libraries(
    test_super_localization_helper
    ${catkin_LIBRARIES}
    localizer_lib
    gmock_main
    super_localizer_helper_lib
  )
  add_dependencies(test_super_localization_helper localizer_lib)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
