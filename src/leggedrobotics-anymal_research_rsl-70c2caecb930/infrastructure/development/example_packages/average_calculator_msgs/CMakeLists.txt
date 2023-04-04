cmake_minimum_required(VERSION 3.5.1)
project(average_calculator_msgs)

set(CATKIN_PACKAGE_DEPENDENCIES)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
    message_generation
)

add_message_files(
  FILES
    Value.msg
)

add_service_files(
  FILES
    GetAverageValue.srv
)

generate_messages(
  DEPENDENCIES
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
    message_runtime
)
