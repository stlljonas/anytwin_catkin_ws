cmake_minimum_required(VERSION 3.1.3)
project(anymal_model_ros)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  any_measurements_ros
  anymal_msgs
  anymal_model
  anymal_description
  kindr_ros
  param_io
  robot_utils_ros
  romo
  series_elastic_actuator_msgs
  series_elastic_actuator_ros
  std_utils
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${kindr_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/conversion_traits/AnymalJointStatesRosConversionTraits.cpp
  src/conversion_traits/AnymalModelRosConversionTraits.cpp
  src/conversion_traits/AnymalStateRosConversionTraits.cpp
  src/conversion_traits/ExtendedAnymalStateRosConversionTraits.cpp
  src/initialization_traits/ActuatorCommandsRosInitializationTraits.cpp
  src/initialization_traits/ActuatorReadingsRosInitializationTraits.cpp
  src/initialization_traits/ActuatorStatesRosInitializationTraits.cpp
  src/initialization_traits/AnymalContactsRosInitializationTraits.cpp
  src/initialization_traits/AnymalFrameTransformsRosInitializationTraits.cpp
  src/initialization_traits/AnymalJointStatesRosInitializationTraits.cpp
  src/initialization_traits/AnymalStateRosInitializationTraits.cpp
  src/initialization_traits/ExtendedActuatorReadingsRosInitializationTraits.cpp
  src/inertial_parameters_ros.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########
## Clang ##
###########

## Note: Do not use CT_WERROR or CT_FIX here.
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_clang_tooling(
    TARGETS
      ${PROJECT_NAME}
    SOURCE_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_FIX
  )
endif(cmake_clang_tools_FOUND)
