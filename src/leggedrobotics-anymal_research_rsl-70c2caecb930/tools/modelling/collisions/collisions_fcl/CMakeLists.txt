cmake_minimum_required(VERSION 3.5.1)
project(collisions_fcl)
add_compile_options(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
  collisions
  collisions_geometry
  # collisions_visualization
  roscpp
  interactive_markers
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(fcl REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   DEPENDS kindr
)

###########
## Build ##
###########

include_directories(include
  ${EIGEN3_INCLUDE_DIR})
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/ColliderManagerFcl.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  fcl
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

#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#
#  add_rostest_gtest(${PROJECT_NAME}-test
#    test/collisions_fcl.test
#    test/conversion_tests.cpp
#    test/multiple_enum_tests.cpp
#    test/colliding_tests.cpp
    # test/interactive_test.cpp
#    test/test_declarations.cpp
#    test/test_main.cpp
#  )
#
#  if(TARGET collisions_fcl-test)
#    add_dependencies(${PROJECT_NAME}-test ${catkin_EXPORTED_TARGETS})
#    target_link_libraries(${PROJECT_NAME}-test
#      ${PROJECT_NAME}
#      ${catkin_LIBRARIES}
#      fcl
#    )
#  endif()
#endif()
