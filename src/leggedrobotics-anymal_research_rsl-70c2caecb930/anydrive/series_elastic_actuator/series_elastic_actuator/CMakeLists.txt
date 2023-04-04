cmake_minimum_required(VERSION 3.5.1)
project(series_elastic_actuator)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    any_measurements
)

add_definitions(-std=c++11)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    any_measurements
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a cpp library
add_library(
  ${PROJECT_NAME}
  src/series_elastic_actuator/common.cpp
  src/series_elastic_actuator/SeActuatorCommand.cpp
  src/series_elastic_actuator/SeActuatorReading.cpp
  src/series_elastic_actuator/SeActuatorReadingExtended.cpp
  src/series_elastic_actuator/SeActuatorState.cpp
  src/series_elastic_actuator/SeActuatorStateExtended.cpp
)
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
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
