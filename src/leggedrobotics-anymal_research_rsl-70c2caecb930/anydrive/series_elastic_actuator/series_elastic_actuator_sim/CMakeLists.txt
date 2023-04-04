cmake_minimum_required(VERSION 3.5.1)
project(series_elastic_actuator_sim)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    series_elastic_actuator
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
    series_elastic_actuator
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
  
  src/series_elastic_actuator_sim/controller/ControllerBase.cpp
  src/series_elastic_actuator_sim/controller/MotorVelocityControllerLQRPosition.cpp
  src/series_elastic_actuator_sim/controller/MotorVelocityControllerPDPosition.cpp
  src/series_elastic_actuator_sim/controller/MotorVelocityControllerTorque.cpp
  src/series_elastic_actuator_sim/controller/MotorVelocityControllerMotorVelocity.cpp
  src/series_elastic_actuator_sim/controller/MotorVelocityControllerZeroMotorVelocity.cpp
  src/series_elastic_actuator_sim/controller/TorqueControllerDisable.cpp
  src/series_elastic_actuator_sim/controller/TorqueControllerFreeze.cpp
  src/series_elastic_actuator_sim/controller/TorqueControllerImpedance.cpp
  src/series_elastic_actuator_sim/controller/TorqueControllerJointPositionVelocityLqr.cpp
  src/series_elastic_actuator_sim/controller/TorqueControllerJointPositionVelocityPid.cpp
  src/series_elastic_actuator_sim/controller/TorqueControllerJointPositionVelocityTorque.cpp
  src/series_elastic_actuator_sim/controller/TorqueControllerJointPositionVelocityTorquePidGains.cpp
  src/series_elastic_actuator_sim/controller/TorqueControllerPDPositionFFTorque.cpp
  src/series_elastic_actuator_sim/controller/TorqueControllerTorque.cpp
  
  src/series_elastic_actuator_sim/hardware/MotorBase.cpp
  src/series_elastic_actuator_sim/hardware/SpringBase.cpp
  src/series_elastic_actuator_sim/hardware/SpringLinear.cpp
  
  src/series_elastic_actuator_sim/SeActuatorBase.cpp
  src/series_elastic_actuator_sim/SeActuatorPerfectTorqueSource.cpp
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
