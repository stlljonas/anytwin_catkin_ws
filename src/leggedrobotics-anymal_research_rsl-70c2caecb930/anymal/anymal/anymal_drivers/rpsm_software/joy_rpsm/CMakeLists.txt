cmake_minimum_required(VERSION 3.5.1)
project(joy_rpsm)

add_definitions(-std=c++11)

find_package(
  catkin REQUIRED
  COMPONENTS
    anymal_msgs
    any_msgs
    joy_manager
    std_srvs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    joy_rpsm
  CATKIN_DEPENDS
    anymal_msgs
    any_msgs
    joy_manager
    std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  joy_rpsm
    src/JoyRpsm.cpp
)

add_dependencies(
  joy_rpsm
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  joy_rpsm
    ${catkin_LIBRARIES}
)

install(TARGETS joy_rpsm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES joy_rpsm.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
