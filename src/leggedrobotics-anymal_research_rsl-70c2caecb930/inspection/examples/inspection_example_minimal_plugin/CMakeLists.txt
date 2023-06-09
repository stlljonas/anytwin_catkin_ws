cmake_minimum_required(VERSION 3.5.1)
project(inspection_example_minimal_plugin)

set(CMAKE_CXX_STANDARD 11)
add_definitions(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  any_inspection_ros
  message_logger
  pluginlib
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/inspection_example_minimal_plugin/InspectionExampleMinimalPlugin.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES any_inspection_ros_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
