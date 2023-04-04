# Project configuration
cmake_minimum_required (VERSION 3.5.1)
project(zmp_optimizer)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  anymal_description
  curves
  numopt_quadprog
  numopt_sqp_line_search
  numopt_common
  motion_generation
  message_logger
  robot_utils
  tinyxml_tools
  loco
  motion_generation_utils
)

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

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS 
    kindr
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
  src/ZmpOptimizerDynamicWalk.cpp
  src/MotionPlan.cpp
  src/SupportPolygon.cpp
  src/LinearZmpFunctionConstraints.cpp
  src/NonLinearZmpFunctionConstraints.cpp
  src/QuadraticZmpObjectiveFunction.cpp
  src/NonLinearZmpObjectiveFunction.cpp
  src/VirtualPlaneFrame.cpp
  src/ZmpParameterHandler.cpp
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
  TARGETS 
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY 
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  # Add gtest based cpp test target and link libraries
  catkin_add_gtest(${PROJECT_NAME}-test
    test/test_main.cpp
    test/zmpOptTest.cpp
    test/NonlinearConstraintsGradientTest.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
  )
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  endif()

  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(TEST_BUILD_TARGETS ${PROJECT_NAME}-test)
  endif(cmake_code_coverage_FOUND)

endif()


###########
## Clang ##
###########
#find_package(cmake_clang_tools QUIET)
#if(cmake_clang_tools_FOUND)
#  add_clang_tooling(
#    TARGETS
#      ${PROJECT_NAME}
#      test_${PROJECT_NAME}
#      # test_${PROJECT_NAME}_ros
#    SOURCE_DIRS
#      ${CMAKE_CURRENT_SOURCE_DIR}/src
#      ${CMAKE_CURRENT_SOURCE_DIR}/include
#      ${CMAKE_CURRENT_SOURCE_DIR}/test
#    CT_HEADER_DIRS
#      ${CMAKE_CURRENT_SOURCE_DIR}/include
#    CF_FIX
#  )
#endif(cmake_clang_tools_FOUND)

