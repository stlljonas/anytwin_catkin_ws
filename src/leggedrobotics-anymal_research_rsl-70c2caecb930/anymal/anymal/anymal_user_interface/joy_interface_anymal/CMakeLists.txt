cmake_minimum_required(VERSION 3.5.1)
project(joy_interface_anymal)

add_definitions(-std=c++11)

find_package(
  catkin REQUIRED
  COMPONENTS
    joy_interface
    sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    joy_interface_anymal
  CATKIN_DEPENDS
    joy_interface
    sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  joy_interface_anymal
    src/AnymalModule.cpp
)

add_dependencies(
  joy_interface_anymal
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  joy_interface_anymal
    ${catkin_LIBRARIES}
)

install(TARGETS joy_interface_anymal
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES joy_interface_anymal.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
