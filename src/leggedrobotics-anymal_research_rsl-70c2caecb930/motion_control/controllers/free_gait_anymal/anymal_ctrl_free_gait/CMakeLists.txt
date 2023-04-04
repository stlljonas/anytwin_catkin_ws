# Project configuration
cmake_minimum_required (VERSION 2.8)
project(anymal_ctrl_free_gait)

add_definitions(--std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  anymal_model
  basic_filters
  robot_utils
  signal_logger
  robot_control
  anymal_motion_control
  loco
  loco_anymal
  numopt_quadprog
  free_gait_core
  free_gait_anymal_common
  grid_map_core
  message_logger
  yaml_tools
  tinyxml_tools
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    anymal_model
    basic_filters
    robot_utils
    signal_logger
    robot_control
    anymal_motion_control
    loco
    loco_anymal
    numopt_quadprog
    free_gait_core
    free_gait_anymal_common
    message_logger
    yaml_tools
    tinyxml_tools
)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${kindr_INCLUDE_DIRS}
)

# Generate compile_commands.json (needed if clang-tidy is run)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(${PROJECT_NAME}
  src/FreeGait.cpp
  src/FreeGaitVirtualModel.cpp
  src/FreeGaitImpedance.cpp
  src/base/AdapterLoco.cpp
  src/base/StateLoco.cpp
  src/base/StepComputerMultiThreaded.cpp
  src/base/GaitPatternFreeGait.cpp
  src/base/LimbCoordinatorFreeGait.cpp
  src/base/FootPlacementStrategyFreeGait.cpp
  src/base/TorsoControlFreeGait.cpp
  src/base/TerrainModelFreeGait.cpp
  src/base/ImpedanceAndVirtualModelControllerFreeGait.cpp
  src/virtual_model/VirtualModelControllerFreeGait.cpp
  src/custom_commands/CustomCommandsManager.cpp
  src/custom_commands/EmptyCommand.cpp
  src/custom_commands/FreezeJointsCommand.cpp
)
add_dependencies(${PROJECT_NAME}
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

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
