cmake_minimum_required(VERSION 3.5.1)
project(anydrive_ethercat)

set(CMAKE_CXX_STANDARD 11)
add_definitions(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PACKAGE_DEPENDENCIES
  anydrive
  soem
  soem_interface
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/setup/AnydriveEthercat.cpp
  src/${PROJECT_NAME}/setup/SetupEthercat.cpp
  src/${PROJECT_NAME}/AnydriveEthercatBusManager.cpp
  src/${PROJECT_NAME}/AnydriveEthercatSlave.cpp
  src/${PROJECT_NAME}/AnydriveManagerEthercat.cpp
)
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(
  ${PROJECT_NAME}_minimal_example
  src/${PROJECT_NAME}_minimal_example.cpp
)
add_dependencies(
  ${PROJECT_NAME}_minimal_example
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}_minimal_example
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(
  ${PROJECT_NAME}_setup_example
  src/${PROJECT_NAME}_setup_example.cpp
)
add_dependencies(
  ${PROJECT_NAME}_setup_example
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}_setup_example
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(
  ${PROJECT_NAME}_controller_example
  src/${PROJECT_NAME}_controller_example.cpp
)
add_dependencies(
  ${PROJECT_NAME}_controller_example
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}_controller_example
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_minimal_example ${PROJECT_NAME}_setup_example ${PROJECT_NAME}_controller_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
