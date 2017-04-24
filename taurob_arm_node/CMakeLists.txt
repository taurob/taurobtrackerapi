cmake_minimum_required(VERSION 2.8.3)
project(taurob_arm_node)

find_package(catkin REQUIRED COMPONENTS
  libtaurob
  roscpp
  sensor_msgs
  std_msgs
  taurob_driver_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(taurob_arm_node
  src/taurob_arm_node.cpp
)

add_dependencies(taurob_arm_node
  taurob_driver_msgs_generate_messages_cpp
)

target_link_libraries(taurob_arm_node
   ${catkin_LIBRARIES}
)
