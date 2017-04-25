cmake_minimum_required(VERSION 2.8.3)
project(libtaurob)

find_package(catkin)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(libtaurob
  src/Arm.cpp
  src/Arm_frames.cpp
  src/Arm_segment.cpp
  src/Base_frames.cpp
  src/libtaurob_base.cpp
  src/SocketTCP.cpp
  src/SocketUDP.cpp
  src/SocketUDPBinary.cpp
)

target_link_libraries(libtaurob
  boost_thread
  boost_system
  boost_date_time
  ${catkin_LIBRARIES}
)
