cmake_minimum_required(VERSION 2.8.3)
project(taurob_watchdog_client)

find_package(catkin)

catkin_package()

add_executable(taurob_watchdog_client
  src/taurob_watchdog_client.cpp
)

target_link_libraries(taurob_watchdog_client
   ${catkin_LIBRARIES}
   pthread
)
