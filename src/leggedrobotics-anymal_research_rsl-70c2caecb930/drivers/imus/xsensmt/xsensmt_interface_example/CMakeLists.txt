# Minimum cmake version
cmake_minimum_required(VERSION 3.5.1)

# Create project
project(xsensmt_interface_example)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    xsensmt_interface
    xsensmt_ros
)

find_package(PkgConfig)
pkg_check_modules(LIBXSENSMT libxsensmt) 

# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if you package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    ${LIBXSENSMT_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    xsensmt_interface
    xsensmt_ros
  DEPENDS
	LIBXSENSMT
)

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${LIBXSENSMT_INCLUDE_DIRS}
)

# add an executable for driver testing
add_executable(
  XsensMTInterfaceTest
  src/XsensMTInterfaceTest.cpp
 )

# Specify libraries to link a library or executable target against
target_link_libraries(
  XsensMTInterfaceTest
  ${catkin_LIBRARIES}
  ${LIBXSENSMT_LIBRARIES}
 )

# add an executable for XsensMTROSInterface node
add_executable(
  xsensmt_node
  src/xsensmt_node.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(
  xsensmt_node
  ${catkin_LIBRARIES}
  ${LIBXSENSMT_LIBRARIES}
)

install(TARGETS xsensmt_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



