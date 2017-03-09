cmake_minimum_required(VERSION 2.8.3)
project(estop)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  sensor_msgs
  laser_geometry
  tf
  footprint_publisher
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(estop_node src/estop.cpp)

target_link_libraries(estop_node
  ${catkin_LIBRARIES}
)
