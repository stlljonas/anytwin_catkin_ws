cmake_minimum_required(VERSION 3.5.1)
project(yaml_tools)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  message_logger
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    yaml-cpp
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/internal/node_to_ordered_string.cpp
  src/${PROJECT_NAME}/internal/scalar_to_string.cpp
  src/${PROJECT_NAME}/YamlNode.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(format_file
  src/format_file.cpp
)
add_dependencies(format_file
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(format_file
  ${PROJECT_NAME}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  TARGETS
    ${PROJECT_NAME} format_file
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/main.cpp
    test/TestYamlNodeEmpty.cpp
    test/TestYamlNodeScalar.cpp
    test/TestYamlNodeScalarBoolFalse.cpp
    test/TestYamlNodeScalarBoolTrue.cpp
    test/TestYamlNodeScalarInteger.cpp
    test/TestYamlNodeScalarDoubleNonInteger.cpp
    test/TestYamlNodeScalarDoubleInteger.cpp
    test/TestYamlNodeScalarDoubleScientific.cpp
    test/TestYamlNodeScalarDoubleInfPos.cpp
    test/TestYamlNodeScalarDoubleInfNeg.cpp
    test/TestYamlNodeScalarDoubleNaN.cpp
    test/TestYamlNodeScalarStringSingleWord.cpp
    test/TestYamlNodeScalarStringMultipleWords.cpp
    test/TestYamlNodeScalarStringEmpty.cpp
    test/TestYamlNodeScalarStringSpecialCharacters.cpp
    test/TestYamlNodeMap.cpp
    test/TestYamlNodeSequenceOfMaps.cpp
    test/TestYamlNodeSequenceOfScalars.cpp
    test/TestYamlNodeNested.cpp
  )
endif()

if(TARGET test_${PROJECT_NAME})
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage()
  endif(cmake_code_coverage_FOUND)
endif()

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling(
    TARGETS
      ${PROJECT_NAME}
      format_file
      test_${PROJECT_NAME}
  )
endif(cmake_clang_tools_FOUND)
