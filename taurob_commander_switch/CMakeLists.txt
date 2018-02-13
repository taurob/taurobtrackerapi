cmake_minimum_required(VERSION 2.8)

project(taurob_commander_switch)

set(MESSAGE_DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

find_package(catkin REQUIRED COMPONENTS
    rospy
    message_generation
    ${MESSAGE_DEPENDENCIES}
)

add_service_files(FILES
    Change.srv
    State.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    CATKIN_DEPENDS
        message_runtime
        ${MESSAGE_DEPENDENCIES}
)
