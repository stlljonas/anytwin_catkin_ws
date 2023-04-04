cmake_minimum_required(VERSION 3.5.1)
project(probabilistic_contact_estimation)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  anymal_model
  basic_contact_estimation
  basic_filters
  signal_logger_msgs
  robot_utils
  grid_map_core
  grid_map_ros
)

find_package(Eigen3 REQUIRED)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    anymal_model
    basic_contact_estimation
    basic_filters
    signal_logger_msgs
    robot_utils
    grid_map_core
    grid_map_ros
)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${kindr_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
    src/State.cpp
    src/StateElevationMap.cpp
    src/measurement_model/ContactProbability.cpp
    src/measurement_model/ContactProbabilityContainer.cpp
    src/measurement_model/ContactProbabilityDynamics.cpp
    src/measurement_model/ContactProbabilityDiffKinematics.cpp
    src/measurement_model/ContactProbabilityKinematics.cpp
    src/measurement_model/SlippingProbabilityDiffKinematics.cpp
    src/transition_model/TransitionProbability.cpp
    src/transition_model/TransitionProbabilityFromContact.cpp
    src/transition_model/TransitionProbabilityFromAir.cpp
    src/transition_model/TransitionProbabilityFromNoSlip.cpp
    src/transition_model/TransitionProbabilityFromSlip.cpp
    src/transition_model/TransitionProbabilityContainer.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED)

  include_directories(
    include
    SYSTEM
      ${catkin_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
      ${kindr_INCLUDE_DIRS}
  )

  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
endif()

###################
## Code_coverage ##
###################
find_package(cmake_code_coverage QUIET)
if(cmake_code_coverage_FOUND)
  add_gtest_coverage(
    TEST_BUILD_TARGETS
      test_${PROJECT_NAME}
  )
endif(cmake_code_coverage_FOUND)