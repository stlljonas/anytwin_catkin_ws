cmake_minimum_required(VERSION 3.3)
project(any_measurements)

set(CMAKE_CXX_STANDARD 11)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS message_logger
  DEPENDS 
    kindr
)

###########
## Build ##
###########
include_directories(
  include
  SYSTEM ${catkin_INCLUDE_DIRS}
)

#add_library(${PROJECT_NAME}
#
#)

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

###########
## Tests ##
###########
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
    ${PROJECT_NAME}_test
    test/${PROJECT_NAME}_test.cpp
    test/TimeTest.cpp
  )
endif()

if(TARGET ${PROJECT_NAME}_test)
  target_link_libraries(${PROJECT_NAME}_test
    ${catkin_LIBRARIES}
  )
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)