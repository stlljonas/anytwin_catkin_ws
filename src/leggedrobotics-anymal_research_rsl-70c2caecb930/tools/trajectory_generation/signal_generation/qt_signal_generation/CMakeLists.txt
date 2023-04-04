cmake_minimum_required(VERSION 3.5.1)
project(qt_signal_generation)

add_definitions(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    qcustomplot
    signal_generation
)
find_package(Qt5Widgets REQUIRED)

set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/qt_signal_visualizer.hpp
  include/${PROJECT_NAME}/SignalVisualizer.hpp
)
set(${PROJECT_NAME}_HDRS_QT
  include/${PROJECT_NAME}/SignalVisualizerWidget.hpp
)
set(${PROJECT_NAME}_SRCS
  src/${PROJECT_NAME}/SignalVisualizer.cpp
  src/${PROJECT_NAME}/SignalVisualizerWidget.cpp
)

catkin_package(
  INCLUDE_DIRS
    include
    ${Qt5Widgets_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
    Qt5::Widgets
  CATKIN_DEPENDS
    qcustomplot
    signal_generation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)
add_definitions(${Qt5Widgets_DEFINITIONS})
qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS_QT})

add_library(
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_MOCS}
)
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

add_executable(
  ${PROJECT_NAME}_example
  src/${PROJECT_NAME}_example.cpp
)
add_dependencies(
  ${PROJECT_NAME}_example
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}_example
  ${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
