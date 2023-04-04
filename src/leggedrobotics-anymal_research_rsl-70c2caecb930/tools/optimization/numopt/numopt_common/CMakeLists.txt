cmake_minimum_required(VERSION 3.5.1)
project(numopt_common)

add_definitions(-std=c++11)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS other_catkin_pkg
)

###########
## Build ##
###########

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(${PROJECT_NAME}
  src/ObjectiveFunction.cpp
  src/NonlinearObjectiveFunction.cpp
  src/WeightedSumNonlinearObjectiveFunction.cpp
  src/QuadraticObjectiveFunction.cpp
  src/WeightedSumQuadraticObjectiveFunction.cpp
  src/L2Regularizer.cpp

  src/FunctionConstraints.cpp
  src/NonlinearFunctionConstraints.cpp
  src/LinearFunctionConstraints.cpp
  
  src/Parameterization.cpp
  src/ParameterizationIdentity.cpp

  src/Problem.cpp
  src/UnconstrainedNonlinearProblem.cpp
  src/ConstrainedNonlinearProblem.cpp
  src/QuadraticProblem.cpp

  src/ConstrainedNonlinearProblemSolver.cpp
  src/QuadraticProblemSolver.cpp

)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
    test/NonlinearObjectiveFunction_test.cpp
    test/NonlinearFunctionConstraints_test.cpp
    test/LinearFunctionConstraints_test.cpp
    test/L2Regularizer_test.cpp
    test/WeightedSumQuadraticObjectiveFunction_test.cpp
  )
  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME}
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )
  endif()
endif()
