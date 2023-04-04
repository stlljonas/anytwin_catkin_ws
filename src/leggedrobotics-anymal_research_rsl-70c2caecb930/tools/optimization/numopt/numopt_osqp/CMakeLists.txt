cmake_minimum_required(VERSION 3.5.1)
project(numopt_osqp)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  numopt_common
  numopt_problems
  osqp
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES numopt_osqp
  CATKIN_DEPENDS
      numopt_common
      numopt_problems
      osqp
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/OperatorSplittingQuadraticProgramFunctionMinimizer.cpp
)

add_dependencies(numopt_osqp ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

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
if (CATKIN_ENABLE_TESTING)
catkin_add_gtest(test_numopt_osqp
    test/test_main.cpp
    test/OperatorSplittingQuadraticProgramSolver_test.cpp
)

target_link_libraries(test_numopt_osqp 
    numopt_osqp 
    ${catkin_LIBRARIES}
)
endif()
