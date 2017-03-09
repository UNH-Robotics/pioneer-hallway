cmake_minimum_required(VERSION 2.8.3)
project(footprint_publisher)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Footprint.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS geometry_msgs roscpp message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(footprint_publisher_node src/footprint_publisher.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(footprint_publisher_node
  ${catkin_LIBRARIES}
)
