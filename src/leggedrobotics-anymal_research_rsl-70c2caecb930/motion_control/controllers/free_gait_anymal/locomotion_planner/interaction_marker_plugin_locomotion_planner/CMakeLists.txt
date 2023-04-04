cmake_minimum_required(VERSION 3.1.3)
project(interaction_marker_plugin_locomotion_planner)

set(CMAKE_CXX_STANDARD 11)

add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
        interaction_marker
        locomotion_planner_msgs
        message_logger
        actionlib
)

find_package(catkin REQUIRED
        COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS
            include
        LIBRARIES
            ${PROJECT_NAME}
        CATKIN_DEPENDS
            ${CATKIN_PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########
include_directories(
        include
        SYSTEM
          ${catkin_INCLUDE_DIRS}
)

add_library(
        ${PROJECT_NAME}
        src/${PROJECT_NAME}/InteractionMarkerPluginLocomotionPlanner.cpp
)
add_dependencies(
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES interaction_marker_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#################
## Clang Tools ##
#################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_FIX
    )
endif(cmake_clang_tools_FOUND)
