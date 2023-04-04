#-------------------------------------------------------------------------------
#
# Author(s): Christian Gehring, Stelian Coros
# Date     : 1-Dec-2014
#
#-------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.5.1)
project(anymal_highlevel_controller)

add_definitions(-std=c++11)

find_package(Boost REQUIRED COMPONENTS thread chrono)

set(PACKAGE_DEPENDENCIES
    any_measurements_ros
    any_msgs
    kindr_ros
    anymal_model
    anymal_model_ros
    anymal_msgs
    anymal_roco
    parameter_handler_ros
    robot_utils
    robot_utils_ros
    roco
    roscpp
    any_node
    series_elastic_actuator_ros
    message_logger
    signal_logger
    signal_logger_ros
    signal_logger_std
    notification
    rocoma_ros
    signal_handler
    cosmo_ros
    param_io
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES}
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

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
  LIBRARIES
    anymal_highlevel_controller
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
)

###########
## Build ##
###########

include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${kindr_INCLUDE_DIRS})


add_library(anymal_highlevel_controller
  src/ModelData.cpp
  src/AnymalHighLevelController.cpp
)
add_dependencies(anymal_highlevel_controller
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(anymal_highlevel_controller
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(anymal_highlevel_controller_node
  src/anymal_highlevel_controller_node.cpp
)
add_dependencies(anymal_highlevel_controller_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(anymal_highlevel_controller_node
  anymal_highlevel_controller
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  # Build unit tests
  catkin_add_gtest(test_${PROJECT_NAME}
      test/test_main.cpp
      test/ModelData_tests.cpp
  )
  add_dependencies(test_${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_${PROJECT_NAME}
      ${PROJECT_NAME}
  )

  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage()
  endif(cmake_code_coverage_FOUND)

endif()

#############
## Install ##
#############

install(TARGETS anymal_highlevel_controller anymal_highlevel_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
