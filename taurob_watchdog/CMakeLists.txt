cmake_minimum_required(VERSION 2.8.3)
project(taurob_watchdog)

find_package(catkin REQUIRED COMPONENTS
  libtaurob
  roscpp
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(taurob_watchdog_node
  src/taurob_watchdog_node.cpp
)

target_link_libraries(taurob_watchdog_node
   boost_system
   ${catkin_LIBRARIES}
)
