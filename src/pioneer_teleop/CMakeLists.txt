cmake_minimum_required(VERSION 2.8.3)
project(pioneer_teleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  rospy
  sensor_msgs
  teleop_twist_joy
  actionlib_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
