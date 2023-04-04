cmake_minimum_required(VERSION 3.5.1)
project(geometry_utils_ros)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CATKIN_PACKAGE_DEPENDENCIES
  any_measurements_ros
  geometry_msgs
  geometry_utils
  kindr_ros
  tf2_ros
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

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
)

###########
## Build ##
###########

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/conversions.cpp
  src/TransformListenerRos.cpp
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
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

##########
## Test ##
##########

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(test_${PROJECT_NAME}
    test/ConversionsTest.cpp
    test/test_main.cpp
    WORKING_DIRECTORY
      ${PROJECT_SOURCE_DIR}/test
  )
  add_rostest_gtest(rostest_${PROJECT_NAME}
    test/geometry_utils_ros.test
    test/rostest_main.cpp
    test/TransformListenerTest.cpp
  )

  if(TARGET test_${PROJECT_NAME} AND TARGET rostest_${PROJECT_NAME})
      target_link_libraries(test_${PROJECT_NAME}
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
      )
      target_link_libraries(rostest_${PROJECT_NAME}
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
      )
      find_package(cmake_code_coverage QUIET)
      if(cmake_code_coverage_FOUND)
        add_gtest_coverage()
        add_rostest_coverage(TEST_BUILD_TARGETS rostest_${PROJECT_NAME})
      endif(cmake_code_coverage_FOUND)
  endif()

endif()


#################
## Clang Tools ##
#################

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
    add_default_clang_tooling(TARGETS ${PROJECT_NAME} test_${PROJECT_NAME} rostest_${PROJECT_NAME})
endif(cmake_clang_tools_FOUND)
