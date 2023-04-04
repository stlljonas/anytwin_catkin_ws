cmake_minimum_required(VERSION 3.5.1)
project(cosmo_ros)

find_package(catkin REQUIRED COMPONENTS
  cosmo
  roscpp
  message_logger
  param_io
  message_generation
  std_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    cosmo
    roscpp
    message_logger
    message_runtime
    param_io
    std_msgs
)

###########
## Build ##
###########
add_definitions(-std=c++11 -fPIC -Wall -Werror)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # Start up.
  add_rostest_gtest(test_${PROJECT_NAME}_node
    test/publisher_ros.test
    test/PublisherRosTest.cpp
    test/test_main.cpp
  )
  
  add_dependencies(test_${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  
  target_link_libraries(test_${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
  )

  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_rostest_coverage()
  endif(cmake_code_coverage_FOUND)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)


#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
