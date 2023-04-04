cmake_minimum_required(VERSION 3.3)
project(inspection_payload_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
)

add_message_files(
  DIRECTORY
    msg
  FILES
    Command.msg
    Joint.msg
    Mode.msg
    State.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    LedCommand.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)