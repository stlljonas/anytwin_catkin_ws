# Project configuration
cmake_minimum_required (VERSION 2.8)
project(anymal_ctrl_free_gait_ros)

add_definitions(--std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  loco_ros_anymal
  locomotion_planner
  anymal_ctrl_free_gait
  anymal_msgs
  free_gait_ros
  curves_ros
  actionlib
  std_srvs
  any_msgs
  kindr_ros
  anymal_motion_control
  robot_control
  pluginlib
  param_io
)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS filesystem)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    anymal_ctrl_free_gait_ros
  CATKIN_DEPENDS
    anymal_ctrl_free_gait
    loco_ros_anymal
    anymal_msgs
    free_gait_ros
    curves_ros
    std_srvs
    any_msgs
    kindr_ros
    anymal_motion_control
    robot_control
    pluginlib
    kindr
  DEPENDS
    Boost
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${kindr_INCLUDE_DIRS}
)

# Generate compile_commands.json (needed if clang-tidy is run)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(anymal_ctrl_free_gait_ros
  src/FreeGaitImpedanceRos.cpp
  src/FreeGaitVirtualModelRos.cpp
)

add_dependencies(anymal_ctrl_free_gait_ros
  ${catkin_EXPORTED_TARGETS}
)

# Add clang tooling to your target
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_FIX
  )
endif(cmake_clang_tools_FOUND)

target_link_libraries(anymal_ctrl_free_gait_ros
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS anymal_ctrl_free_gait_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES robot_control_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
