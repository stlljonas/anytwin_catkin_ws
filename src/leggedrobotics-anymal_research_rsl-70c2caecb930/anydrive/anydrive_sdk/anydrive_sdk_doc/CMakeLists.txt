cmake_minimum_required(VERSION 3.5.1)
project(anydrive_sdk_doc)

find_package(Doxygen)

IF(DOXYGEN)
  find_package(catkin REQUIRED
    COMPONENTS
      anydrive
      anydrive_ethercat
      anydrive_ethercat_ros
      anydrive_monitor
      anydrive_msgs
      anydrive_ros
      rqt_anydrive_monitor
  )

  catkin_package(
    CATKIN_DEPENDS
      anydrive
      anydrive_ethercat
      anydrive_ethercat_ros
      anydrive_monitor
      anydrive_msgs
      anydrive_ros
      rqt_anydrive_monitor
  )

  # Set environment variables.
  set(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  set(DOXYGEN_IMAGE_DIRS "\"${CMAKE_CURRENT_SOURCE_DIR}/figures/\"" )
  set(DOXYGEN_SOURCE_DIRS "\"${CMAKE_CURRENT_SOURCE_DIR}/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../anydrive/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../anydrive_ethercat/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../anydrive_ethercat_ros/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../anydrive_monitor/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../anydrive_msgs/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../anydrive_ros/\"")
  set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../rqt_anydrive_monitor/\"")
  set(HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
  MAKE_DIRECTORY(${HTML_DIR})

  # Add configuration file.
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.config.in ${DOCSOURCE_DIR}/doxygen.config IMMEDIATE)

  # Build doxygen.
  execute_process(COMMAND ${DOXYGEN} ${DOCSOURCE_DIR}/doxygen.config)

  # Unset environment variables.
  unset(${DOCSOURCE_DIR} CACHE)
  unset(${DOXYGEN_IMAGE_DIRS} CACHE)
  unset(${DOXYGEN_SOURCE_DIRS} CACHE)
  unset(${HTML_DIR} CACHE)

ELSE(DOXYGEN)
  message(FATAL_ERROR "Doxygen not found. Install it with: sudo apt install doxygen")
ENDIF(DOXYGEN)

