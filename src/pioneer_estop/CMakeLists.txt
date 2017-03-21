cmake_minimum_required(VERSION 2.8.3)
project(pioneer_teleop)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  sensor_msgs
  std_srvs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
