cmake_minimum_required (VERSION 2.8)
project(lightweight_filtering_models)
add_definitions(-std=c++11)

##################### Set some variables #####################
if(DEFINED CATKIN_DEVEL_PREFIX)
else()
SET(CMAKE_BUILD_TYPE "Release")
endif()
##################### Find, include, and compile library #####################

find_package(catkin REQUIRED COMPONENTS)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

if(DEFINED CATKIN_DEVEL_PREFIX)
	find_package(catkin REQUIRED COMPONENTS
  		lightweight_filtering
  		roscpp
	)
	message(STATUS "Building lightweight_filtering_models as catkin package")
  include_directories(${catkin_INCLUDE_DIRS})
else()
  find_package(kindr REQUIRED)
  include_directories(${kindr_INCLUDE_DIRS})
  find_package(LWF REQUIRED)
  include_directories(${LWF_INCLUDE_DIRS})
endif()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

if(DEFINED CATKIN_DEVEL_PREFIX)
	catkin_package(
	   	INCLUDE_DIRS include
  		CATKIN_DEPENDS lightweight_filtering roscpp
  	)
endif()

if(DEFINED CATKIN_DEVEL_PREFIX)
	add_executable(lwfm_test src/lwfm_test.cpp)
	target_link_libraries(lwfm_test ${catkin_LIBRARIES})
endif()

# Generate FindLWFM.cmake file
file(WRITE cmake/FindLWFM.cmake
"# This file was automatically generated during the installation of the lightweight_filtering_models library
# and can be used through cmake to find the corresponding header files. A copy of this
# file was created in ${CMAKE_ROOT}/Modules (depending on the CMAKE_ROOT variable).

set(LWFM_INCLUDE_DIRS
${CMAKE_INSTALL_PREFIX}/include/LWFM/include
)
set(LWFM_FOUND TRUE)
message(\"-- LWFM found (include: ${CMAKE_INSTALL_PREFIX}/include/LWFM/include)\")
"
)

# Setting for make install
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_INSTALL_PREFIX}/include/LWFM)")
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES cmake/FindLWFM.cmake DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
