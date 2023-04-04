cmake_minimum_required(VERSION 3.5.1)
project(anymal_lowlevel_controller)

set(PACKAGE_DEPENDENCIES
    any_measurements_ros
    any_msgs
    any_node
    anydrive
    anydrive_ethercat
    anydrive_ethercat_ros
    anymal_lowlevel_controller_common
    anymal_model_ros
    anymal_msgs
    conditional_state_machine
    message_logger
    romo_std
    roscpp
    std_msgs
    roslib
    cosmo_ros
    signal_logger_ros
    series_elastic_actuator_anydrive
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
)


# Generate postinst
set(LLC_EXECUTABLE_PATH ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/${PROJECT_NAME}_node)
configure_file(debian/postinst.in ${CMAKE_CURRENT_SOURCE_DIR}/debian/postinst @ONLY)

# Build
add_definitions(-std=c++11)

# set RPATH, such that the libraries can be found without the need to call `ldconfig`
# see https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling for details
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/anymal_lowlevel_controller/state_machine/StateMachine.cpp
  src/anymal_lowlevel_controller/AnymalLowLevelController.cpp
  src/anymal_lowlevel_controller/System.cpp
)
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(
  ${PROJECT_NAME}_node
  src/anymal_lowlevel_controller_node.cpp
)
add_dependencies(
  ${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Prevent the linker from removing transitive dependencies (see Readme).
set_target_properties(${PROJECT_NAME}_node PROPERTIES LINK_FLAGS "-Wl,--no-as-needed")

# Install
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
