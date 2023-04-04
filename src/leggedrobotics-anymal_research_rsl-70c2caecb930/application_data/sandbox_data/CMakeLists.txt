cmake_minimum_required(VERSION 3.5.1)
project(sandbox_data)

find_package(catkin REQUIRED)

catkin_package()

# Custom target will always cause its dependencies to be evaluated and is
# run by default.
add_custom_target(dummy_target ALL
  DEPENDS
    download_output
)

## Download data.
# download_output will always be rebuilt because it depends on always_rebuild.
set(PATH "${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "Checking and downloading data if needed ...")
add_custom_command(
  OUTPUT download_output
  COMMAND bin/download.sh ${PATH} ${PROJECT_NAME} anymal_research gJyPebM5X74Re4PM7aB8
  WORKING_DIRECTORY ${PATH}
  DEPENDS
    always_rebuild
)

# Dummy output which is never actually produced. Anything that depends on
# this will always be rebuilt.
add_custom_command(
  OUTPUT always_rebuild
  COMMAND cmake -E echo
)

# Create symlinks for Gazebo to find the meshes and models.
if (NOT IS_SYMLINK ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/meshes)
  execute_process(
    COMMAND ln -s ${CMAKE_SOURCE_DIR}/meshes
      ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/meshes
  )
endif()
if (NOT IS_SYMLINK ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/models)
  execute_process(
    COMMAND ln -s ${CMAKE_SOURCE_DIR}/models
      ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/models
  )
endif()

# Install data.
install(
  DIRECTORY
    actions
    audio
    config
    environments
    meshes
    missions
    models
    point_clouds
    rviz
    staircase_descriptions
    waypoints
    worlds
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
  FILES
    actions.yaml
    collections.yaml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
