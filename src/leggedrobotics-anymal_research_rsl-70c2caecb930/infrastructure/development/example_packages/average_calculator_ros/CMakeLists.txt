cmake_minimum_required(VERSION 3.5.1)
project(average_calculator_ros)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_HEADER_DEPENDENCIES
  average_calculator
  average_calculator_msgs
  roscpp
)

set(CATKIN_PACKAGE_DEPENDENCIES
  ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
  message_logger
  param_io
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/AverageCalculatorRos.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

##########
## Test ##
##########

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    COMPONENTS
      ${CATKIN_PACKAGE_DEPENDENCIES}
      roslaunch
      rostest
  )

  # Run a ROS test (add rostest test-dependency).
  # The *.test file has to be on the top of the list.
  add_rostest_gtest(test_${PROJECT_NAME}_node
    test/ros_interface.test
    test/RosInterface.cpp
  )
  # Test all launch files for sanity (add roslaunch test-dependency).
  # You can also test a single file using `launch/my_file.launch`.
  roslaunch_add_file_check(launch)
endif()

if(TARGET test_${PROJECT_NAME}_node)
  target_link_libraries(test_${PROJECT_NAME}_node
    gtest_main
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  target_include_directories(test_${PROJECT_NAME}_node PRIVATE
    include
  )

  target_include_directories(test_${PROJECT_NAME}_node SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )

  # Generate test coverage report -> ${PROJECT_BINARY_DIR}/cmake_code_coverage/${PROJECT_NAME}/index.html
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
      add_rostest_coverage()
  endif(cmake_code_coverage_FOUND)
endif()


###########
## Clang ##
###########

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
