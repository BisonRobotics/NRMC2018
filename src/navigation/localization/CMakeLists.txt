cmake_minimum_required(VERSION 2.8.3)
project(localization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
  tf2_ros
  tf
)

find_package (Eigen3 REQUIRED)
find_package (vesc_access REQUIRED)
find_package (sensor_access REQUIRED)
find_package (teleop_skid_steer REQUIRED)

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
  INCLUDE_DIRS include/ 
  LIBRARIES localizer_lib super_localizer_lib
#  CATKIN_DEPENDS nav_msgs roscpp
  DEPENDS EIGEN3 vesc_access sensor_access
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${vesc_access_INCLUDE_DIRS}
  ${sensor_access_INCLUDE_DIRS}
  include/
  ${teleop_skid_steer_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(localizer_lib src/localizer/localizer.cpp)
add_dependencies(localizer_lib vesc_access lpresearch_imu )
target_link_libraries(localizer_lib vesc_access lpresearch_imu apriltag_tracker_interface)

add_library(super_localizer_helper_lib src/super_localizer/super_localizer_helper.cpp)
target_link_libraries(super_localizer_helper_lib localizer_lib)
add_dependencies(super_localizer_helper_lib localizer_lib apriltag_tracker_interface)

add_library(super_localizer_lib src/super_localizer/super_localizer.cpp)
add_dependencies(super_localizer_lib localizer_lib apriltag_tracker_interface)
target_link_libraries(super_localizer_lib localizer_lib super_localizer_helper_lib)

add_library(PoseEstimateFilter src/pose_estimate_filter/pose_estimate_filter.cpp)
target_link_libraries(PoseEstimateFilter ${Eigen3_LIBS})
target_link_libraries(PoseEstimateFilter ${catkin_LIBRARIES})
target_link_libraries(PoseEstimateFilter ${OpenCV_LIBS})

############
# Programs #
############
add_executable(localizer_teleop_test tests/localization_test.cpp)
add_dependencies(localizer_teleop_test apriltag_tracker_interface)
#target_link_libraries (apriltag_tracker_interface ${catkin_LIBRARIES})
target_link_libraries(localizer_teleop_test ${catkin_LIBRARIES})
target_link_libraries(localizer_teleop_test lpresearch_imu)
target_link_libraries(localizer_teleop_test apriltag_tracker_interface)
target_link_libraries(localizer_teleop_test super_localizer_lib)
target_link_libraries(localizer_teleop_test teleop_interface)

add_executable(pose_estimate_filter src/pose_estimate_filter/pose_estimate_filter_node.cpp)
target_link_libraries(pose_estimate_filter ${catkin_LIBRARIES})
target_link_libraries(pose_estimate_filter PoseEstimateFilter)


add_executable(test_imu_orientation tests/imu_test.cpp)
target_link_libraries(test_imu_orientation ${catkin_LIBRARIES} lpresearch_imu)
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

catkin_add_gtest(pose_estimate_filter_tests ../localization/tests/pose_estimate_filter/pose_estimate_filter_tests.cpp)
if(TARGET pose_estimate_filter_tests)
  target_link_libraries(pose_estimate_filter_tests ${catkin_LIBRARIES})
  target_link_libraries(pose_estimate_filter_tests PoseEstimateFilter)
endif()

