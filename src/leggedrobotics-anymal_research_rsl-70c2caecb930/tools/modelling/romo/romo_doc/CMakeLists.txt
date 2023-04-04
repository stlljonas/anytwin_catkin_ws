cmake_minimum_required(VERSION 3.5.1)
project(romo_doc)

#--------------------------#
# Documentation

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(Doxygen)

IF(DOXYGEN)
  find_package(catkin REQUIRED COMPONENTS
    romo
    romo_std
    romo_rbdl
  )

  catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS
      romo
      romo_std
      romo_rbdl
    DEPENDS doxygen
  )


  # Set some environment variables
  set(DOXYGEN_SOURCE_DIRS "\"${CMAKE_CURRENT_SOURCE_DIR}/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../romo/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../romo_std/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../romo_rbdl/\"")
  Set(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  Set(DOXYGEN_IMAGE_DIRS "\"${CMAKE_CURRENT_SOURCE_DIR}/figures/\"" )
  Set(HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
  MAKE_DIRECTORY(${HTML_DIR})

  # Add configure file
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.config.in ${DOCSOURCE_DIR}/doxygen.config IMMEDIATE)

  # Build doxygen
  execute_process(COMMAND ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)
  #ADD_CUSTOM_TARGET(doc ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)

  # Unset cache files
  unset(${DOXYGEN_SOURCE_DIRS} CACHE)

ELSE(DOXYGEN)
  message(FATAL_ERROR "Doxygen not found")
ENDIF(DOXYGEN)
