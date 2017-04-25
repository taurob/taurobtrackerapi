cmake_minimum_required(VERSION 2.8.3)
project(taurob_base_node)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  geometry_msgs
  libtaurob
  roscpp
  sensor_msgs
  std_msgs
  std_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(taurob_base_node
  src/taurob_base_node.cpp
)

target_link_libraries(taurob_base_node
  ${catkin_LIBRARIES}
)
