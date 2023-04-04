cmake_minimum_required(VERSION 3.5.1)
project(numopt_sqp)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  numopt_common
  numopt_problems
  numopt_quadprog
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES numopt_sqp
  CATKIN_DEPENDS numopt_common
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(numopt_sqp
 src/SQPFunctionMinimizer.cpp
)

add_dependencies(numopt_sqp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
catkin_add_gtest(test_numopt_sqp
  test/test_main.cpp
  #test/Quadprog_test.cpp
)

target_link_libraries(test_numopt_sqp
    numopt_sqp
    ${catkin_LIBRARIES}
)
