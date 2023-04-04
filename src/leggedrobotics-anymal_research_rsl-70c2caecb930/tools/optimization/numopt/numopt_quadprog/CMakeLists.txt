cmake_minimum_required(VERSION 3.5.1)
project(numopt_quadprog)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  quadprog_catkin
  numopt_common
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS quadprog_catkin numopt_common
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/ActiveSetFunctionMinimizer.cpp
)

target_link_libraries( ${PROJECT_NAME}
  ${catkin_LIBRARIES}
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
  find_package(numopt_problems REQUIRED)

  include_directories(${numopt_problems_INCLUDE_DIRS})

  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
    test/ActiveSetFunctionMinimizer_test.cpp
  )

  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME}
      numopt_quadprog
      ${catkin_LIBRARIES}
      ${numopt_problems_LIBRARIES}
    )
  endif()
endif()
