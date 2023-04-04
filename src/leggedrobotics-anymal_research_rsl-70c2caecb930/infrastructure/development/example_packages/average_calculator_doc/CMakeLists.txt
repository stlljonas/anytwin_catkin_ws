cmake_minimum_required(VERSION 3.5.1)
project(average_calculator_doc)

find_package(Doxygen)

IF(DOXYGEN)
  set(CATKIN_PACKAGE_DEPENDENCIES
    average_calculator
    average_calculator_ros
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
  set(DOXYGEN_PROJECT_NAME "Average Calculator")
  set(DOXYGEN_PROJECT_NUMBER "${average_calculator_VERSION}")
  set(DOXYGEN_PROJECT_BRIEF "Library for calculating averages.")
  set(DOXYGEN_IMAGE_PATH "\"${CMAKE_CURRENT_SOURCE_DIR}/../\"")
  set(DOXYGEN_INPUT "\"${CMAKE_CURRENT_SOURCE_DIR}/\"")
  set(DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${CMAKE_CURRENT_SOURCE_DIR}/../README.md\"")
  set(DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${CMAKE_CURRENT_SOURCE_DIR}/../average_calculator/\"")
  set(DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${CMAKE_CURRENT_SOURCE_DIR}/../average_calculator/README.md\"")
  set(DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${CMAKE_CURRENT_SOURCE_DIR}/../average_calculator_ros/\"")
  set(DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${CMAKE_CURRENT_SOURCE_DIR}/../average_calculator_ros/README.md\"")
  set(DOXYGEN_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  set(DOXYGEN_OUTPUT_HTML_DIRECTORY ${DOXYGEN_OUTPUT_DIRECTORY}/html)

  # Add configuration file.
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.config.in ${DOXYGEN_OUTPUT_DIRECTORY}/doxygen.config IMMEDIATE)

  # Create an html build folder.
  MAKE_DIRECTORY(${DOXYGEN_OUTPUT_HTML_DIRECTORY})

  # Build doxygen.
  add_custom_target(
    ${PROJECT_NAME} ALL
    COMMAND ${DOXYGEN} ${DOXYGEN_OUTPUT_DIRECTORY}/doxygen.config
    WORKING_DIRECTORY ${DOXYGEN_OUTPUT_DIRECTORY}
    COMMENT "${PROJECT_NAME}: Generating API documentation."
    VERBATIM
  )

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
