cmake_minimum_required(VERSION 2.8.3)
project(disable_motors)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
