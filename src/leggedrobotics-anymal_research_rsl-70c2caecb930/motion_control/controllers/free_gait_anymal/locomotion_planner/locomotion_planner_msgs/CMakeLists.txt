cmake_minimum_required(VERSION 3.5.1)
project(locomotion_planner_msgs)

find_package(catkin REQUIRED
        COMPONENTS
        message_generation
        actionlib_msgs
        geometry_msgs
        std_msgs
)

add_action_files(
        FILES
        NavigateToGoalPose.action
)

generate_messages(
        DEPENDENCIES
        actionlib_msgs
        geometry_msgs
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        actionlib_msgs
        geometry_msgs
        message_runtime
        std_msgs
)