cmake_minimum_required (VERSION 2.8)
project(loco_anymal)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  loco
  loco_sea
  anymal_motion_control
  anymal_roco
  anymal_description
  anymal_model
  cmake_modules
  romo_measurements
  whole_body_control_romo
  kindr
)

find_package(catkin REQUIRED
  COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Find system dependencies providing a CMake module.
find_package(Eigen3 REQUIRED)
find_package(TinyXML REQUIRED)

###################################
## catkin specific configuration ##
###################################

# Declare this project as a catkin package
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    loco
    loco_sea
    anymal_motion_control
    anymal_roco
    anymal_description
    anymal_model
    romo_measurements
    whole_body_control_romo
    kindr
  DEPENDS
    TinyXML
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  # Robot measurements implementation
  src/common/CommandTranslator.cpp
  src/common/LegPropertiesAnymal.cpp
  src/common/LegAnymal.cpp
  src/common/LegsAnymal.cpp
  src/common/PointFootAnymal.cpp

  # Motion control
  src/motion_control/ImpedanceAndVirtualModelController.cpp
  src/motion_control/ImpedanceController.cpp
  src/motion_control/WholeBodyController.cpp

  # Heading generation
  src/heading_generation/HeadingGeneratorAnymal.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${TinyXML_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation.
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY test/resources/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test/resources/
)

##########
## Test ##
##########

# Add tests
if(CATKIN_ENABLE_TESTING)
  ## Find catkin dependencies, including test dependencies.
  find_package(catkin REQUIRED
    COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  )

  ## Add gtest based cpp test target and link libraries.
  ## Note that the tests are executed in the order of the list of source files.
  ## Therefore, put basic tests in the top and more advanced tests in the bottom.
  catkin_add_gtest(test_${PROJECT_NAME}
    test/GaitPatternTest.cpp
    test/TerrainPerceptionFreePlaneTest.cpp
    test/JointLimitsTest.cpp
  )
endif()

# Link tests
if(TARGET test_${PROJECT_NAME})
  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  target_include_directories(test_${PROJECT_NAME} PRIVATE
    include
  )

  target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )

  #add_dependencies(test_${PROJECT_NAME}
  #  ${catkin_EXPORTED_TARGETS}
  #)

  # Generate test coverage report -> ${PROJECT_BINARY_DIR}/cmake_code_coverage/${PROJECT_NAME}/index.html
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
      TEST_BUILD_TARGETS test_${PROJECT_NAME}
    )
  endif(cmake_code_coverage_FOUND)
endif()
