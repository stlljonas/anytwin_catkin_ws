cmake_minimum_required(VERSION 3.5.1)
project(basic_filters_doc)

find_package(Doxygen)

IF(DOXYGEN)
  set(CATKIN_PACKAGE_DEPENDENCIES
    basic_filters
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
  set(DOXYGEN_PROJECT_NAME "Basic Filters")
  set(DOXYGEN_PROJECT_NUMBER "${basic_filters_VERSION}")
  set(DOXYGEN_PROJECT_BRIEF "Library implementing basic filters.")
  set(DOXYGEN_IMAGE_PATH "\"${CMAKE_CURRENT_SOURCE_DIR}/../\"")
  set(DOXYGEN_INPUT "\"${CMAKE_CURRENT_SOURCE_DIR}/\"")
  set(DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${CMAKE_CURRENT_SOURCE_DIR}/../README.md\"")
  set(DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${CMAKE_CURRENT_SOURCE_DIR}/../basic_filters/\"")
  set(DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${CMAKE_CURRENT_SOURCE_DIR}/../basic_filters/README.md\"")
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
