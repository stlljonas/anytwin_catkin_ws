cmake_minimum_required(VERSION 3.1.3)
project(anymal_model)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  analytical_inverse_kinematics
  anymal_description
  message_logger
  robot_utils
  romo
  romo_rbdl
  romo_std
  series_elastic_actuator
  signal_logger
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS filesystem)
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
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
    kindr
)

###########
## Build ##
###########

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${kindr_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/actuator_containers.cpp
  src/AnymalModel.cpp
  src/AnymalParameters.cpp
  src/AnymalState.cpp
  src/dynamics_utils.cpp
  src/AnymalInverseKinematics.cpp
  src/LegConfigurations.cpp
  src/LimitsAnymal.cpp
  src/rbdl_utils.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

##########
## Test ##
##########

if(CATKIN_ENABLE_TESTING)
  ## Find catkin dependencies, including test dependencies.
  find_package(catkin REQUIRED
    COMPONENTS
      ${CATKIN_PACKAGE_DEPENDENCIES}
      romo_test
  )
  include_directories(
    include
    SYSTEM
      ${catkin_INCLUDE_DIRS}
  )

  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
    test/romo_tests.cpp
    test/AnymalState_tests.cpp
    test/inverse_kinematics_tests.cpp
    test/KinematicsWorldToBase_tests.cpp
#    test/KinematicsWorldToFoot_tests.cpp
#    test/KinematicsWorldToBody_tests.cpp
#    test/KinematicsWorldToCom_tests.cpp
#    test/Dynamics_tests.cpp
    test/starleth_kinematics/jacobiansLeftForeStarleth.cpp
#    test/rbdl_utils_tests.cpp
    test/identification_tests.cpp
  )
endif()

if(TARGET test_${PROJECT_NAME})
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
    ${CMAKE_SOURCE_DIR}/test/starleth_dynamics/starleth_dynamics.a
  )
endif()

###########
## Clang ##
###########

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_clang_tooling(
    TARGETS
      ${PROJECT_NAME}
      test_${PROJECT_NAME}
    SOURCE_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/test
    CT_HEADER_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_FIX
  )
endif(cmake_clang_tools_FOUND)
