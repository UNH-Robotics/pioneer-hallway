cmake_minimum_required(VERSION 2.8.3)
project(pioneer_test)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pioneer3dx_tf_broadcaster src/pioneer3dx_tf_broadcaster.cpp)
target_link_libraries(pioneer3dx_tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(pioneer3dx_tf_broadcaster ${catkin_EXPORTED_TARGETS})

add_executable(pioneer3dx_odometry_publisher src/pioneer3dx_odometry_publisher.cpp)
target_link_libraries(pioneer3dx_odometry_publisher ${catkin_LIBRARIES})
add_dependencies(pioneer3dx_odometry_publisher ${catkin_EXPORTED_TARGETS})

add_executable(odom_map_frame_publisher src/odom_map_frame_publisher.cpp)
target_link_libraries(odom_map_frame_publisher ${catkin_LIBRARIES})
add_dependencies(odom_map_frame_publisher ${catkin_EXPORTED_TARGETS})

add_executable(laser_tf_broadcaster src/laser_tf_broadcaster.cpp)
target_link_libraries(laser_tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(laser_tf_broadcaster ${catkin_EXPORTED_TARGETS})

add_executable(rplidar_laser_publisher src/rplidar_laser_publisher.cpp)
target_link_libraries(rplidar_laser_publisher ${catkin_LIBRARIES})
add_dependencies(rplidar_laser_publisher ${catkin_EXPORTED_TARGETS})

add_executable(rplidar_laser_republisher src/rplidar_laser_republisher.cpp)
target_link_libraries(rplidar_laser_republisher ${catkin_LIBRARIES})
add_dependencies(rplidar_laser_republisher ${catkin_EXPORTED_TARGETS})

add_executable(pioneer3dx_pid_controller src/pioneer3dx_pid_controller.cpp)
target_link_libraries(pioneer3dx_pid_controller ${catkin_LIBRARIES})
add_dependencies(pioneer3dx_pid_controller ${catkin_EXPORTED_TARGETS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
