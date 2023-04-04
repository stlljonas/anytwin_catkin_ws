# Project configuration
cmake_minimum_required (VERSION 2.8)
project(hierarchical_optimization)

# Setup dependencies
set(PKG_DEPENDENCIES
  std_utils
  signal_logger
  parameter_handler
  message_logger
)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS ${PKG_DEPENDENCIES})

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

find_package(Eigen3 REQUIRED)

# Declare this project as a catkin package
catkin_package(
 INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
 LIBRARIES hierarchical_optimization
 CATKIN_DEPENDS 
     std_utils
     signal_logger
     parameter_handler
     message_logger
 DEPENDS kindr
)

add_definitions(--std=c++11)

# Includes
include_directories(include) 
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${kindr_INCLUDE_DIRS})

# Add library
add_library(hierarchical_optimization
    src/HierarchicalOptimizationBase.cpp
    src/HierarchicalOptimizationNullSpaceProjection.cpp
    src/HierarchicalOptimizationQpNullSpaceProjection.cpp
    src/HierarchicalOptimizationQpCascade.cpp
)

add_dependencies(hierarchical_optimization ${catkin_EXPORTED_TARGETS})
target_link_libraries(hierarchical_optimization ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(numopt_quadprog REQUIRED)
  include_directories(${numopt_quadprog_INCLUDE_DIRS})

  catkin_add_gtest(test_hierarchical_optimization
    test/test_main.cpp
    test/HierarchicalOptimizationNullSpaceProjectionTest.cpp
    test/HierarchicalOptimizationQpNullSpaceProjectionTest.cpp
  )
  if(TARGET test_hierarchical_optimization)
    target_link_libraries(test_hierarchical_optimization
      ${catkin_LIBRARIES}
      hierarchical_optimization 
      ${numopt_quadprog_LIBRARIES}
    )
  endif()

  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(TEST_BUILD_TARGETS test_hierarchical_optimization)
  endif(cmake_code_coverage_FOUND)

endif()

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
