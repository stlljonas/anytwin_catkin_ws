cmake_minimum_required(VERSION 3.5.1)
project(conditional_state_machine)

add_definitions(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    message_logger
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    message_logger
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
