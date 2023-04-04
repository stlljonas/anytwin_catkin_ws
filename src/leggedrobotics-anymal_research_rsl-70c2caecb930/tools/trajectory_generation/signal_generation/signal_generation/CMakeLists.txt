cmake_minimum_required(VERSION 3.5.1)
project(signal_generation)

add_definitions(-std=c++11)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(
  ${PROJECT_NAME}_example
  src/${PROJECT_NAME}_example.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_example
  ${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  # Tests in order of their dependency.
  catkin_add_gtest(
    ${PROJECT_NAME}_test
    test/RangeTest.cpp
    test/SingleProfileTest.cpp
    test/SingleProfileDerivativesTest.cpp
    test/CombinedProfilesTest.cpp
    test/SamplingTest.cpp
    test/signal_generation_test.cpp
  )
endif()

if(TARGET ${PROJECT_NAME}_test)
  target_link_libraries(
    ${PROJECT_NAME}_test
    ${PROJECT_NAME}
  )
endif()
