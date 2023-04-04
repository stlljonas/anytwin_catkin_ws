cmake_minimum_required(VERSION 3.5.1)
project(inspection_example_msgs)

set(CATKIN_PACKAGE_DEPENDENCIES
  actionlib_msgs
  environment_item_msgs
  geometry_msgs
  message_generation
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

add_message_files(
  DIRECTORY
    msg
  FILES
    Item.msg
)

add_action_files(
  DIRECTORY
    action
  FILES
    InspectItem.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    environment_item_msgs
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    environment_item_msgs
    geometry_msgs
    message_runtime
    std_msgs
)