cmake_minimum_required(VERSION 3.5.1)
project(anymal_roco)

set(PACKAGE_DEPENDENCIES
    any_measurements
    message_logger
    anymal_description
    anymal_model
    roco
    signal_logger
)

find_package(catkin REQUIRED
  COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

add_definitions(-std=c++14)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    anymal_roco
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
    kindr
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(anymal_roco
  src/anymal_roco.cpp
  src/checks/ContactStateCheck.cpp
  src/checks/JointPositionLimitsCheck.cpp
  src/checks/JointPositionsCheck.cpp
  src/checks/StateStatusCheck.cpp
  src/RocoCommand.cpp
  src/RocoState.cpp
  src/StateChecker.cpp
)

add_dependencies(anymal_roco
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(anymal_roco
  ${catkin_LIBRARIES}
)

install(TARGETS anymal_roco
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
