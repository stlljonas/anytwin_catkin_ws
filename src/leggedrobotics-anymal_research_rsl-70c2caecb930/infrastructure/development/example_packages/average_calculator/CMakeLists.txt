######################################################################
## Best Practice CMakeLists.txt                                     ##
##                                                                  ##
## Based on the CMakeLists.txt created by catkin_create_pkg.        ##
## See http://wiki.ros.org/catkin/CMakeLists.txt for documentation. ##
## Delete unused blocks and comments in your package.               ##
######################################################################

## Set up the CMake project.
## Use at least 3.5.1 (Ubuntu 16.04, last LTS).
## In case you want to use newer features, use 3.10.2 (Ubuntu 18.04).
cmake_minimum_required(VERSION 3.5.1)
project(average_calculator)

## Set the C++ standard.
## Use at least 11 (ROS Kinetic).
## In case you want to use newer features, use 14 (ROS Melodic).
## Source: https://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
set(CMAKE_CXX_STANDARD 11)

## Uncomment below to build this package in debug mode. However, this is only meant for
## use during development and should be commented in general since, for this package,
## it overwrites the build type specified in the catkin workspace configuration.
## Source: https://cmake.org/cmake/help/latest/variable/CMAKE_BUILD_TYPE.html
# set(CMAKE_BUILD_TYPE Debug)

## Enable all compiler warnings and treat them as errors.
## -Wall, -Wextra and -Wpedantic are mandatory, -Werror must not be used.
## -Wall:      Enable a wide range of compiler warnings.
## -Wextra:    Enable even more compiler warnings (though still not all of them).
## -Wpedantic: Warnings demanded by strict ISO C and ISO C++ (nonexhaustive).
## -Werror:    Transform all warnings into errors.
##             Note: To prevent excessive warnings from external libraries, the
##                   include_directories option 'SYSTEM' should be used. (Example below)
## Source: https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
add_compile_options(-Wall -Wextra -Wpedantic)

## Export compile commands for clang.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Configure the message logger.
## MELO_USE_COUT:        Change the MELO backend from ROS to std::cout (not recommended).
## MELO_FUNCTION_PRINTS: Add the name of the function to the output (only useful with std::cout backend, not recommended).
## MELO_MIN_SEVERITY:    When set to MELO_SEVERITY_INFO, debug output is removed during compilation (recommended for time critical applications).
## Source: https://cmake.org/cmake/help/latest/command/add_definitions.html
#add_definitions(-DMELO_USE_COUT -DMELO_FUNCTION_PRINTS -DMELO_MIN_SEVERITY=MELO_SEVERITY_INFO)

## Create a list of catkin package dependencies used in header files of this package.
set(CATKIN_PACKAGE_HEADER_DEPENDENCIES
#  roscpp
#  std_msgs
)

## Create a list of catkin package dependencies, now for both header and source files.
set(CATKIN_PACKAGE_DEPENDENCIES
  ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
#  message_logger
)

## Find catkin dependencies for building this package.
find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
#    message_generation
)

## Find system dependencies providing a CMake module.
#find_package(Boost REQUIRED COMPONENTS system)
#find_package(Eigen3 REQUIRED)

## Find system dependencies not providing a CMake module, but a pkg-config file.
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(GST REQUIRED gstreamer-1.0)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed.
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder.
#add_message_files(
#  FILES
#    Message1.msg
#    Message2.msg
#)

## Generate services in the 'srv' folder.
#add_service_files(
#  FILES
#    Service1.srv
#    Service2.srv
#)

## Generate actions in the 'action' folder.
#add_action_files(
#  FILES
#    Action1.action
#    Action2.action
#)

## Generate added messages and services with any dependencies listed here.
#generate_messages(
#  DEPENDENCIES
#    std_msgs  # Or other packages containing msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder.
#generate_dynamic_reconfigure_options(
#  cfg/DynReconf1.cfg
#  cfg/DynReconf2.cfg
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package.
## Declare things to be passed to dependent projects.
## INCLUDE_DIRS:   Header files which downstream dependencies require to build. These normally consist of the "include"
##                 folder of this package, as well as header files from system dependencies which do not follow the
##                 CMake naming convention.
## LIBRARIES:      Libraries which downstream dependencies require to build. These normally consist of the libraries you
##                 build in this package, as well as libraries from system dependencies which do not follow the CMake
##                 naming convention. Libraries which contain plugins must not be added.
## CATKIN_DEPENDS: Catkin packages which downstream package require to build (they will call find_package on these).
##                 These normally consist of the catkin packages which this package includes in its header files. Catkin
##                 packages which this package solely includes in its source files must not be added. For more
##                 information, see https://answers.ros.org/question/58498/what-is-the-purpose-of-catkin_depends/.
## DEPENDS:        System dependencies which downstream dependencies require to build (they will call find_package on
##                 these). System dependencies which this package solely includes in its sources files must not be
##                 added. System dependencies must follow the CMake naming convention (appending *_INCLUDE_DIRS
##                 respectively *_LIBRARIES to their name, e.g. Boost -> Boost_INCLUDE_DIRS/Boost_LIBRARIES). If they do
##                 not (e.g. Eigen3 -> EIGEN3_INCLUDE_DIR) or cannot be found using find_package (e.g. GST requiring
##                 pkg-config), declare them in 'INCLUDE_DIRS' and 'LIBRARIES'.
## CFG_EXTRAS:     CMake flags to be passed to downstream packages, stored in ./cmake folder.
catkin_package(
  INCLUDE_DIRS
    include
#    ${EIGEN3_INCLUDE_DIR} # Eigen does not set variables according to the CMake naming convention.
#    ${GST_INCLUDE_DIRS}   # GST does not set variables according to the CMake naming convention.
  LIBRARIES
    ${PROJECT_NAME}
#    ${GST_LIBRARIES}      # GST does not set variables according to the CMake naming convention.
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
#    message_runtime       # message_runtime is a special case, directly put it here.
#  DEPENDS
#    Boost                 # Boost sets variables according to the CMake naming convention.
#  CFG_EXTRAS
#    extras.cmake
)

###########
## Build ##
###########

## Declare a C++ library.
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/AverageCalculator.cpp
)

## Add CMake target dependencies of the library.
## This could be code that may need to be generated before libraries,
## either from message generation or dynamic reconfigure.
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

## Specify additional locations of header files of the library.
## Your package locations should be listed before other locations.
## Target specific includes are required such that overlays of test dependencies are properly resolved.

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
#  ${GST_LIBRARIES}
)

#############
## Install ##
#############

## All install targets should use catkin DESTINATION variables.
## See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html.

## Mark executable scripts (Python, shell etc.) for installation.
## In contrast to setup.py, you can choose the destination.
#install(PROGRAMS
#  scripts/my_script
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

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

## Mark folders for installation (e.g. config and launch folders).
#install(
#  DIRECTORY config launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

## Mark other files for installation.
#install(
#  FILES myfile1 myfile2
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

##########
## Test ##
##########

## GTest.
if(CATKIN_ENABLE_TESTING)
  ## Find catkin dependencies, including test dependencies.
  find_package(catkin REQUIRED
    COMPONENTS
      ${CATKIN_PACKAGE_DEPENDENCIES}
#      my_catkin_testing_suite
  )

  ## Add gtest based cpp test target and link libraries.
  ## Note that the tests are executed in the order of the list of source files.
  ## Therefore, put basic tests in the top and more advanced tests in the bottom.
  catkin_add_gtest(test_${PROJECT_NAME}
    test/TestAverageCalculator.cpp
  )

  catkin_add_gtest(fixture_test_${PROJECT_NAME}
    test/TestAverageCalculatorUsingFixture.cpp
  )
endif()

## Link GTest.
if(TARGET test_${PROJECT_NAME} AND TARGET fixture_test_${PROJECT_NAME})
  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    ${PROJECT_NAME}
#    ${my_catkin_testing_suite_LIBRARIES}
  )

  target_link_libraries(fixture_test_${PROJECT_NAME}
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

  target_include_directories(fixture_test_${PROJECT_NAME} PRIVATE
    include
  )

  target_include_directories(fixture_test_${PROJECT_NAME} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )

  # Generate test coverage report -> ${PROJECT_BINARY_DIR}/cmake_code_coverage/${PROJECT_NAME}/index.html
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
      add_gtest_coverage(
        TEST_BUILD_TARGETS test_${PROJECT_NAME} fixture_test_${PROJECT_NAME}
      )
  endif(cmake_code_coverage_FOUND)
endif()

## ROS test.
#if(CATKIN_ENABLE_TESTING)
  ## Find catkin dependencies, including test dependencies.
#  find_package(catkin REQUIRED
#    COMPONENTS
#      ${CATKIN_PACKAGE_DEPENDENCIES}
#      rostest
#  )
#  include_directories(
#    include
#    ${catkin_INCLUDE_DIRS}
#  )

  ## Add rostest based cpp test target and link libraries.
#  add_rostest_gtest(
#    test_${PROJECT_NAME}_ros
#    test/ros_interface.test
#    test/main.cpp
#    test/RosInterface.cpp
#  )
#endif()

## Link ROS test.
#if(TARGET test_${PROJECT_NAME}_ros)
#  target_link_libraries(
#    test_${PROJECT_NAME}_ros
#    ${catkin_LIBRARIES}
#    ${rostest_LIBRARIES}
#  )
#endif()

###########
## Clang ##
###########

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
