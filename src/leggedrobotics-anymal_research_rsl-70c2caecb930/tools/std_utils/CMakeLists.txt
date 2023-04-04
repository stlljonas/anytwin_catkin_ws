cmake_minimum_required(VERSION 3.5.1)
project(std_utils)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra)

set(CATKIN_PACKAGE_DEPENDENCIES )

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
)

###########
## Build ##
###########

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_ctm_example
  src/containers/CompileTimeMapExample.cpp
)

add_dependencies(${PROJECT_NAME}_ctm_example
  ${catkin_EXPORTED_TARGETS}
)

add_executable(${PROJECT_NAME}_macro_example
  src/macros/macro_example.cpp
)

add_dependencies(${PROJECT_NAME}_macro_example
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

##########
## Test ##
##########

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
    test/containers/CompileTimeMapTest.cpp
    test/containers/CompileTimeSetTest.cpp
    test/containers/CompileTimeKeysTest.cpp
    test/containers/EnumArrayTest.cpp
    test/containers/KeyArrayTest.cpp
    test/containers/MultiKeyContainerTest.cpp
    test/containers/SmartPointerContainer.cpp
    test/remove_first_if_test.cpp
    WORKING_DIRECTORY
      ${PROJECT_SOURCE_DIR}/test
  )

  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )

    # Generate test coverage report
    find_package(cmake_code_coverage QUIET)
    if(cmake_code_coverage_FOUND)
      add_gtest_coverage()
    endif(cmake_code_coverage_FOUND)
  endif()
endif()

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#################
## Clang Tools ##
#################

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
