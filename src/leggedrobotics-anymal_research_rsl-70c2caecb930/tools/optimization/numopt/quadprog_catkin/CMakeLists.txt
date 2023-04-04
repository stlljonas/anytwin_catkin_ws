# Project configuration
cmake_minimum_required (VERSION 2.8)

project(quadprog_catkin)

# Find Catkin
find_package(catkin REQUIRED)

find_package(Eigen3 REQUIRED)


set(EXECUTABLE_OUTPUT_PATH  ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH     ${PROJECT_SOURCE_DIR}/lib)

# Noisily default to Release build
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
 LIBRARIES quadprog_catkin
)


###########
## Build ##
###########
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
add_definitions(--std=c++11)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

add_library(quadprog_catkin SHARED 
#    src/eQuadProg++.c
    src/ASQP.cpp
)

add_dependencies(quadprog_catkin ${catkin_EXPORTED_TARGETS})
target_link_libraries(quadprog_catkin ${catkin_LIBRARIES})

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
