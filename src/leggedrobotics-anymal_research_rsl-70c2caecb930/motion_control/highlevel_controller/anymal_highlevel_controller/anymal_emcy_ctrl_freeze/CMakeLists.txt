cmake_minimum_required(VERSION 3.5.1)
project(anymal_emcy_ctrl_freeze)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  anymal_model
  loco
  robot_utils
  roco
  rocoma_plugin
  anymal_roco
  tinyxml_tools
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES anymal_emcy_ctrl_freeze
  CATKIN_DEPENDS
    anymal_model
    loco
    robot_utils
    roco
    rocoma_plugin
    anymal_roco
    tinyxml_tools
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

add_definitions(--std=c++11)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
   src/AnymalEmcyCtrlFreeze.cpp
   src/AnymalEmcyCtrlFailproof.cpp
   src/AnymalEmcyCtrlHybridFreeze.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES rocoma_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
