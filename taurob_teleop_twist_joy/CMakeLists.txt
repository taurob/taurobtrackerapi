cmake_minimum_required(VERSION 2.8.3)
project(taurob_teleop_twist_joy)
#set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp roslaunch roslint sensor_msgs trajectory_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs trajectory_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

#add_library(${PROJECT_NAME} src/teleop_twist_joy)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/teleop_node.cpp src/teleop_twist_joy.cpp src/simple_inverse_kinematics.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_node
  PROPERTIES OUTPUT_NAME teleop_node PREFIX "")

#install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#install(DIRECTORY launch config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

roslint_cpp()
roslint_add_test()
roslaunch_add_file_check(launch/teleop.launch)
