# Project configuration
cmake_minimum_required (VERSION 2.8)
project(whole_body_control)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS 
  hierarchical_optimization
  std_utils
  robot_utils
  tinyxml_tools
  parameter_handler
)

find_package(Eigen3 REQUIRED)

# Declare this project as a catkin package
catkin_package(
 INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS 
   hierarchical_optimization
   std_utils
   robot_utils
   tinyxml_tools
   parameter_handler
)

add_definitions(--std=c++11)

# Includes
include_directories(include) 
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

# Add library
add_library(${PROJECT_NAME}
  src/Task.cpp
  src/TaskManager.cpp
  src/TaskSetup.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Install library
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
