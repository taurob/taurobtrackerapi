cmake_minimum_required(VERSION 2.8)

project(taurob_relay_dropper)

set(MESSAGE_DEPENDENCIES
    std_srvs
)

find_package(catkin REQUIRED COMPONENTS
    rospy
    message_generation
    ${MESSAGE_DEPENDENCIES}
)

catkin_package(
    CATKIN_DEPENDS
        message_runtime
        ${MESSAGE_DEPENDENCIES}
)
