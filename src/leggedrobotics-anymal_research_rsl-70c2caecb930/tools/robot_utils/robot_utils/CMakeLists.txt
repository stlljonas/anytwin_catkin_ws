# Project configuration
cmake_minimum_required(VERSION 3.1.3)
project(robot_utils)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  basic_filters
  message_logger
  parameter_handler
  std_utils
  cmake_modules
)

# Find Catkin
find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

find_package(Eigen3 REQUIRED)
find_package(TinyXML REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    kindr
    TinyXML
)

###########
## Build ##
###########
include_directories (
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${TinyXML_INCLUDE_DIRS}
    ${kindr_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  # Function Approximators
  src/function_approximators/dynamicMovementPrimitive/DiscreteMovementPrimitive.cpp
  src/function_approximators/dynamicMovementPrimitive/DmpParameters.cpp
  src/function_approximators/dynamicMovementPrimitive/GaussianKernel.cpp
  src/function_approximators/dynamicMovementPrimitive/GaussianKernelApproximator.cpp
  src/function_approximators/dynamicMovementPrimitive/GaussianKernelJumpPropagator.cpp
  src/function_approximators/dynamicMovementPrimitive/TimelessGaussianKernelApproximator.cpp
  src/function_approximators/dynamicMovementPrimitive/TransformationSystem.cpp

  src/function_approximators/gaussianBaseTrajectory/GaussianBaseTrajectory.cpp

  src/function_approximators/polyharmonicSplines/RBF1D.cpp
  src/function_approximators/polyharmonicSplines/PeriodicRBF1DC3.cpp
  src/function_approximators/polyharmonicSplines/PeriodicRBF1DC1.cpp
  src/function_approximators/polyharmonicSplines/BoundedRBF1D.cpp

  # Function Generators
  src/function_generators/FunctionGeneratorBase.cpp
  src/function_generators/FctLinChirp.cpp
  src/function_generators/FctLogChirp.cpp

  # Geometry
  src/geometry/common/geometry_utils.cpp
  src/geometry/Polygon.cpp
  src/geometry/Line.cpp
  src/geometry/Triangle.cpp
  src/geometry/Tetragon.cpp

  # Interpolators
  src/interpolators/BasicFifthOrderInterpolation.cpp
  src/interpolators/LinearInterpolation.cpp

  # Kinematics
  src/kinematics/kinematics.cpp

  # Math
  src/math/LinearAlgebra.cpp
  src/math/Stochastics.cpp
  src/math/Transformations.cpp

  # Sensors
  src/sensors/Joystick.cpp
  src/sensors/GNSS.cpp

  # Surfaces
  src/surfaces/Surface.cpp
  src/surfaces/Ellipsoid.cpp

  src/force_calibrators/ForceCalibratorBase.cpp
  src/force_calibrators/AverageForceCalibrator.cpp
  src/force_calibrators/NoneForceCalibrator.cpp

  src/physical_definitions.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  # Add gtest based cpp test target and link libraries
  catkin_add_gtest(${PROJECT_NAME}-test
    test/test_main.cpp
    test/schedule/ScheduleTest.cpp
    test/geometry/GeometryTest.cpp
    test/math/LinearAlgebraTest.cpp
    test/math/TransformationsTest.cpp
    test/math/statistics_test.cpp
    test/force_calibrators/AverageForceCalibratorTest.cpp
    test/interpolators/InterpolationTest.cpp
    test/interpolators/InterpolatorTest.cpp
    test/kinematics/kinematicsTest.cpp
    test/sensors/GNSSTest.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
  )
endif()
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
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
    SOURCE_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/test
    CT_HEADER_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_FIX
  )
endif(cmake_clang_tools_FOUND)
