## Set up the CMake project.
cmake_minimum_required(VERSION 3.5.1)
project(loco_perf)

## Compile as C++14
set(CMAKE_CXX_STANDARD 14)

## Uncomment below to build this package in debug mode.
# set(CMAKE_BUILD_TYPE Debug)

## Enable all compiler warnings and treat them as errors.
add_compile_options(-Wall -Wextra -Wpedantic)

## Export compile commands for clang.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Create a list of catkin dependencies.
set(CATKIN_PACKAGE_DEPENDENCIES
  basic_filters
  loco
  loco_anymal
  signal_logger
  tinyxml_tools
)

## Find catkin dependencies for building and execution.
find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

## Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  ## Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package.
## Declare things to be passed to dependent projects.
## See also https://answers.ros.org/question/58498/what-is-the-purpose-of-catkin_depends/.
## INCLUDE_DIRS: uncomment this if your package contains header files.
## LIBRARIES: libraries you create in this project that dependent projects also need.
## CATKIN_DEPENDS: catkin_packages dependent projects also need.
## DEPENDS: system dependencies of this project that dependent projects also need.
## CFG_EXTRAS: cmake flags of this project that dependent projects also need, stored in ./cmake folder.
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

## Specify additional locations of header files.
include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${kindr_INCLUDE_DIRS}
)

## Declare a C++ library.
add_library(${PROJECT_NAME}
  src/PerformanceMonitor.cpp
)

## Add CMake target dependencies of the library.
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# Specify your packages header files.
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  )

# Header directories listed after 'SYSTEM' are marked as external, and don't generate warnings.
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  #  ${Boost_INCLUDE_DIRS}
  #  ${EIGEN3_INCLUDE_DIR}
  #  ${GST_INCLUDE_DIRS}
  )

## Specify libraries to link executable targets against.
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation.
install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation.
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

##########
## Test ##
##########

## GTest.
if(CATKIN_ENABLE_TESTING)
  ## Find catkin dependencies, including test dependencies.
  find_package(catkin REQUIRED
    COMPONENTS
      ${CATKIN_PACKAGE_DEPENDENCIES}
    )

  ## Add gtest based cpp test target and link libraries.
  ## Note that the tests are executed in the order of the list of source files.
  ## Therefore, put basic tests in the top and more advanced tests in the bottom.
  catkin_add_gtest(test_${PROJECT_NAME}
    test/TestPerformanceMonitor.cpp
    )
endif()

## Link GTest.
if(TARGET test_${PROJECT_NAME})
  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    ${PROJECT_NAME}
    #    ${my_catkin_testing_suite_LIBRARIES}
    )

  target_include_directories(test_${PROJECT_NAME} PRIVATE
    include
    )

  target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    )

  # Generate test coverage report -> ${PROJECT_BINARY_DIR}/cmake_code_coverage/${PROJECT_NAME}/index.html
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(TEST_BUILD_TARGETS test_${PROJECT_NAME})
  endif(cmake_code_coverage_FOUND)
endif()


###########
## Clang ##
###########

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
