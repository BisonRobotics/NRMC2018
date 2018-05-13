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
  message_generation
  vesc_access 
  localization
  geometry_msgs
  visualization_msgs
  tf2
  new_sim
  imperio
)

link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

add_action_files(DIRECTORY action FILES 
    distance.action
    waypoint.action 
)
generate_messages(DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES Wheels velocity_plugins
    CATKIN_DEPENDS roscpp pluginlib message_runtime
    #DEPENDS EIGEN3
)

include_directories (include ${catkin_INCLUDE_DIRS} )
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${vesc_access_INCLUDE_DIRS})
include_directories(${localization_INCLUDE_DIRS})
include_directories(${new_sim_INCLUDE_DIRS})
## Declare a C++ library

add_library(velocity_plugins
        src/differential_drive_controller/differential_drive_controller.cpp
        src/wheels/wheels.cpp
)

target_link_libraries(velocity_plugins ${catkin_LIBRARIES})

add_library(Wheels src/wheels/wheels.cpp)

add_library(pos_control src/position_controller/position_controller.cpp)
target_link_libraries (pos_control vesc_access)

add_library(way_control_helper src/waypoint_controller/waypoint_controller_helper.cpp)

add_library(way_control src/waypoint_controller/waypoint_controller.cpp)
add_dependencies(way_control vesc_access way_control_helper)
target_link_libraries (way_control vesc_access way_control_helper)

add_executable (wheel_goal_master src/wheel_goal_master.cpp)
target_link_libraries (wheel_goal_master ${catkin_LIBRARIES} pos_control localizer_lib)

add_executable(waypoint_master src/waypoint_master.cpp)
add_dependencies(waypoint_master way_control sim_robot)
target_link_libraries (waypoint_master way_control ${new_sim_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(waypoint_filter src/waypoint_filter.cpp)
add_dependencies(waypoint_filter way_control_helper)
target_link_libraries(waypoint_filter way_control_helper ${catkin_LIBRARIES})

add_library(super_waypoint_filter src/super_waypoint_filter/super_waypoint_filter.cpp)
add_executable(super_waypoint_master src/super_waypoint_master.cpp)
add_dependencies(super_waypoint_master super_waypoint_filter)
target_link_libraries(super_waypoint_master super_waypoint_filter ${catkin_LIBRARIES})

add_executable(pose_array_publisher src/test/pose_array_publisher.cpp)
target_link_libraries(pose_array_publisher ${catkin_LIBRARIES})

add_dependencies (wheel_goal_master localizer_lib)
add_dependencies (waypoint_master ${PROJECT_NAME}_gencpp)
## Tests

catkin_add_gtest(test_WaypointControllerHelper include/waypoint_controller/waypoint_controller_helper.h src/waypoint_controller/test/waypoint_controller_helper_tests.cpp)
if(TARGET test_WaypointControllerHelper)
  target_link_libraries(test_WaypointControllerHelper
    ${catkin_LIBRARIES}
    gmock_main
	way_control_helper
  )
endif()

catkin_add_gtest(test_WaypointControllerHelper2 include/waypoint_controller/waypoint_controller_helper.h src/waypoint_controller/test/waypoint_controller_helper_tests2.cpp)
if(TARGET test_WaypointControllerHelper2)
  target_link_libraries(test_WaypointControllerHelper2
    ${catkin_LIBRARIES}
    gmock_main
	way_control_helper
  )
endif()

catkin_add_gtest(test_WaypointController include/waypoint_controller/waypoint_controller.h src/waypoint_controller/test/waypoint_controller_tests.cpp)
if(TARGET test_WaypointController)
  target_link_libraries(test_WaypointController
    ${catkin_LIBRARIES}
    way_control
	way_control_helper
    gmock_main
  )
  add_dependencies(test_WaypointController way_control_helper way_control vesc_access)
endif()

catkin_add_gtest(test_Wheels src/wheels/test/wheels_tests.cpp src/wheels/wheels.cpp)
if(TARGET test_Wheels)
  target_link_libraries(test_Wheels
    ${catkin_LIBRARIES}
  )
endif()


catkin_add_gtest(test_SuperWaypointFilter src/super_waypoint_filter/test/super_waypoint_filter_tests.cpp src/super_waypoint_filter/super_waypoint_filter.cpp)
if(TARGET test_SuperWaypointFilter)
  target_link_libraries(test_SuperWaypointFilter
    super_waypoint_filter
    ${catkin_LIBRARIES}
  )
endif()


catkin_add_gtest(test_Interface src/interface/test/interface_tests.cpp include/wheel_control/interface/interface.h src/wheels/wheels.cpp)
if(TARGET test_Interface)
  target_link_libraries(test_Interface
    ${catkin_LIBRARIES}
  )
endif()

catkin_add_gtest(test_velocity_plugins src/differential_drive_controller/test/differential_drive_controller_tests.cpp)
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
    gmock_main
  )
  add_dependencies(test_position_controller vesc_access)
endif()


