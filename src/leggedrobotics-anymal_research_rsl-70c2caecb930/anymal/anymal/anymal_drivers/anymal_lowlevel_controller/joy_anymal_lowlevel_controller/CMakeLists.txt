cmake_minimum_required(VERSION 3.5.1)
project(joy_anymal_lowlevel_controller)

add_definitions(-std=c++11)

find_package(
  catkin REQUIRED
  COMPONENTS
    anymal_lowlevel_controller_common
    anymal_msgs
    any_msgs
    joy_manager
    std_srvs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    joy_anymal_lowlevel_controller
  CATKIN_DEPENDS
    anymal_lowlevel_controller_common
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
  joy_anymal_lowlevel_controller
    src/JoyAnymalLowlevelController.cpp
)

add_dependencies(
  joy_anymal_lowlevel_controller
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  joy_anymal_lowlevel_controller
    ${catkin_LIBRARIES}
)

install(TARGETS joy_anymal_lowlevel_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES joy_anymal_lowlevel_controller.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
