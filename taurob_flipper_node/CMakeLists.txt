cmake_minimum_required(VERSION 2.8.3)
project(taurob_flipper_node)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  libtaurob
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(taurob_flipper_node
  src/taurob_flipper_node.cpp
)

target_link_libraries(taurob_flipper_node
   ${catkin_LIBRARIES}
)
