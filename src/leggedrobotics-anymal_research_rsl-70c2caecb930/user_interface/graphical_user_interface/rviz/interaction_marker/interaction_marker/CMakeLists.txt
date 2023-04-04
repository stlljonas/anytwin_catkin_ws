cmake_minimum_required(VERSION 3.5.1)
project(interaction_marker)

set(CMAKE_CXX_STANDARD 11)

## Source: https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
add_compile_options(-Wall -Wextra -Wpedantic)

## Export compile commands for clang.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    param_io
    pluginlib
    roscpp
    std_srvs
    visualization_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    interactive_markers
    param_io
    pluginlib
    roscpp
    std_srvs
    visualization_msgs
)

add_library(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/InteractionMarker.cpp
  src/${PROJECT_NAME}/InteractionMarkerPluginBase.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(
  ${PROJECT_NAME}_node
  src/interaction_marker_node.cpp
)

add_dependencies(
  ${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  include
)

target_include_directories(${PROJECT_NAME}_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

install(DIRECTORY config launch meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


###########
## Clang ##
###########

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
