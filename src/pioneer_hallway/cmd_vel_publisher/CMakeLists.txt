cmake_minimum_required(VERSION 2.8.3)
project(cmd_vel_publisher)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  std_srvs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
