cmake_minimum_required(VERSION 3.5.1)
project(loco_doc)

find_package(Doxygen)

IF(DOXYGEN)
  set(CATKIN_PACKAGE_DEPENDENCIES
    loco
    loco_anymal
    loco_perf
    loco_ros
    loco_ros_anymal
    loco_sea
    )

  find_package(catkin REQUIRED
    COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
    )

  catkin_package(
    CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  )

  # Set environment variables.
  set(DOXYGEN_PROJECT_NAME "Locomotion Control Library")
  set(DOXYGEN_PROJECT_NUMBER "${loco_doc_VERSION}")
  set(DOXYGEN_PROJECT_BRIEF "Libraries for locomotion control.")
  set(DOXYGEN_IMAGE_PATH "\"${CMAKE_CURRENT_SOURCE_DIR}/../\"")
  set(DOXYGEN_INPUT "\"${CMAKE_CURRENT_SOURCE_DIR}/\"")
  set(DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${CMAKE_CURRENT_SOURCE_DIR}/../README.md\"")
  set(DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${CMAKE_CURRENT_SOURCE_DIR}/../loco/\"")
  set(DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${CMAKE_CURRENT_SOURCE_DIR}/../loco/README.md\"")
  set(DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${CMAKE_CURRENT_SOURCE_DIR}/../loco_anymal/\"")
  set(DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${CMAKE_CURRENT_SOURCE_DIR}/../loco_anymal/README.md\"")
  set(DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${CMAKE_CURRENT_SOURCE_DIR}/../loco_perf/\"")
  set(DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${CMAKE_CURRENT_SOURCE_DIR}/../loco_perf/README.md\"")
  set(DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${CMAKE_CURRENT_SOURCE_DIR}/../loco_ros/\"")
  set(DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${CMAKE_CURRENT_SOURCE_DIR}/../loco_ros_anymal/\"")
  set(DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${CMAKE_CURRENT_SOURCE_DIR}/../loco_sea/\"")
  Set(DOXYGEN_IMAGE_DIRS "\"${CMAKE_CURRENT_SOURCE_DIR}/figures/\"" )
  set(DOXYGEN_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  set(DOXYGEN_OUTPUT_HTML_DIRECTORY ${DOXYGEN_OUTPUT_DIRECTORY}/html)

  # Add configuration file.
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.config.in ${DOXYGEN_OUTPUT_DIRECTORY}/doxygen.config IMMEDIATE)

  # Create an html build folder.
  MAKE_DIRECTORY(${DOXYGEN_OUTPUT_HTML_DIRECTORY})

  # Build doxygen.
  execute_process(COMMAND ${DOXYGEN} ${DOXYGEN_OUTPUT_DIRECTORY}/doxygen.config)

  # Unset environment variables.
  unset(${DOXYGEN_PROJECT_NAME} CACHE)
  unset(${DOXYGEN_PROJECT_NUMBER} CACHE)
  unset(${DOXYGEN_PROJECT_BRIEF} CACHE)
  unset(${DOXYGEN_IMAGE_PATH} CACHE)
  unset(${DOXYGEN_INPUT} CACHE)
  unset(${DOXYGEN_OUTPUT_DIRECTORY} CACHE)
  unset(${DOXYGEN_OUTPUT_HTML_DIRECTORY} CACHE)
ELSE(DOXYGEN)
  message(FATAL_ERROR "Doxygen not found. Install it with: sudo apt install doxygen")
ENDIF(DOXYGEN)