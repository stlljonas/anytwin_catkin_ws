cmake_minimum_required(VERSION 3.5.1)
project(anydrive)

set(CMAKE_CXX_STANDARD 11)
add_definitions(-Wall -Wextra -Wpedantic)
add_definitions("-DMINIMAL_ANYDRIVE_FIRMWARE_VERSION=\"0.0.11\"")
add_definitions("-DNEWEST_ANYDRIVE_FIRMWARE_VERSION=\"0.1.0\"")
add_definitions(-DMELO_MIN_SEVERITY=MELO_SEVERITY_INFO)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PACKAGE_DEPENDENCIES
  any_worker
  any_measurements
  message_logger
  signal_handler
  yaml_tools
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/calibration/parameter/Calibration.cpp
  src/${PROJECT_NAME}/calibration/parameter/FrictionEstimation.cpp
  src/${PROJECT_NAME}/calibration/parameter/GearAndJointEncoderHoming.cpp
  src/${PROJECT_NAME}/calibration/parameter/GearJointEncoderOffset.cpp
  src/${PROJECT_NAME}/calibration/parameter/ImuGyroscopeDcBias.cpp
  src/${PROJECT_NAME}/calibration/parameter/MotorEncoderOffset.cpp
  src/${PROJECT_NAME}/calibration/parameter/MotorEncoderParameters.cpp
  src/${PROJECT_NAME}/calibration/parameter/SpringStiffness.cpp
  src/${PROJECT_NAME}/calibration/routine/CalibrationBase.cpp
  src/${PROJECT_NAME}/calibration/routine/FrictionEstimation.cpp
  src/${PROJECT_NAME}/calibration/routine/GearJointEncoderOffset.cpp
  src/${PROJECT_NAME}/calibration/routine/GravityCompensation.cpp
  src/${PROJECT_NAME}/calibration/routine/SafeJointVelocity.cpp
  src/${PROJECT_NAME}/calibration/CalibrationModeEnum.cpp
  src/${PROJECT_NAME}/calibration/CalibrationState.cpp
  src/${PROJECT_NAME}/calibration/CalibrationTypeEnum.cpp
  src/${PROJECT_NAME}/common/FirmwareInfo.cpp
  src/${PROJECT_NAME}/common/Limits.cpp
  src/${PROJECT_NAME}/common/Macros.cpp
  src/${PROJECT_NAME}/common/Version.cpp
  src/${PROJECT_NAME}/communication/CommunicationInterfaceBase.cpp
  src/${PROJECT_NAME}/communication/CommunicationManagerBase.cpp
  src/${PROJECT_NAME}/configuration/Configuration.cpp
  src/${PROJECT_NAME}/fsm/Controlword.cpp
  src/${PROJECT_NAME}/fsm/StateBase.cpp
  src/${PROJECT_NAME}/fsm/StateCalibrate.cpp
  src/${PROJECT_NAME}/fsm/StateColdStart.cpp
  src/${PROJECT_NAME}/fsm/StateConfigure.cpp
  src/${PROJECT_NAME}/fsm/StateControlOp.cpp
  src/${PROJECT_NAME}/fsm/StateDeviceMissing.cpp
  src/${PROJECT_NAME}/fsm/StateEnum.cpp
  src/${PROJECT_NAME}/fsm/StateError.cpp
  src/${PROJECT_NAME}/fsm/StateFatal.cpp
  src/${PROJECT_NAME}/fsm/StateMachine.cpp
  src/${PROJECT_NAME}/fsm/StateMotorOp.cpp
  src/${PROJECT_NAME}/fsm/StateMotorPreOp.cpp
  src/${PROJECT_NAME}/fsm/StateStandby.cpp
  src/${PROJECT_NAME}/fsm/StateWarmStart.cpp
  src/${PROJECT_NAME}/mode/ModeBase.cpp
  src/${PROJECT_NAME}/mode/ModeCurrent.cpp
  src/${PROJECT_NAME}/mode/ModeDemoChangeGravity.cpp
  src/${PROJECT_NAME}/mode/ModeDemoSafeJointVelocity.cpp
  src/${PROJECT_NAME}/mode/ModeDisable.cpp
  src/${PROJECT_NAME}/mode/ModeEnum.cpp
  src/${PROJECT_NAME}/mode/ModeFreeze.cpp
  src/${PROJECT_NAME}/mode/ModeGearPosition.cpp
  src/${PROJECT_NAME}/mode/ModeGearVelocity.cpp
  src/${PROJECT_NAME}/mode/ModeJointPosition.cpp
  src/${PROJECT_NAME}/mode/ModeJointVelocity.cpp
  src/${PROJECT_NAME}/mode/ModeJointPositionVelocity.cpp
  src/${PROJECT_NAME}/mode/ModeJointPositionVelocityTorque.cpp
  src/${PROJECT_NAME}/mode/ModeJointPositionVelocityTorquePidGains.cpp
  src/${PROJECT_NAME}/mode/ModeJointTorque.cpp
  src/${PROJECT_NAME}/mode/ModeJointVelocity.cpp
  src/${PROJECT_NAME}/mode/ModeMotorPosition.cpp
  src/${PROJECT_NAME}/mode/ModeMotorVelocity.cpp
  src/${PROJECT_NAME}/setup/Anydrive.cpp
  src/${PROJECT_NAME}/setup/JointPositionConfigurationManager.cpp
  src/${PROJECT_NAME}/setup/Setup.cpp
  src/${PROJECT_NAME}/setup/Anydrive.cpp
  src/${PROJECT_NAME}/Anydrive.cpp
  src/${PROJECT_NAME}/AnydriveManager.cpp
  src/${PROJECT_NAME}/Command.cpp
  src/${PROJECT_NAME}/DriveInfo.cpp
  src/${PROJECT_NAME}/JointPositionConfigurationManager.cpp
  src/${PROJECT_NAME}/State.cpp
  src/${PROJECT_NAME}/StateExtended.cpp
  src/${PROJECT_NAME}/Statusword.cpp
  src/${PROJECT_NAME}/thread_sleep.cpp
)
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(
  parse_statusword
  src/parse_statusword.cpp
)
add_dependencies(
  parse_statusword
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  parse_statusword
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} parse_statusword
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY example_setups
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  # Tests in order of their dependency.
  catkin_add_gtest(
    ${PROJECT_NAME}_test
    test/VersionTest.cpp
    test/${PROJECT_NAME}_test.cpp
  )
endif()

if(TARGET ${PROJECT_NAME}_test)
  target_link_libraries(
    ${PROJECT_NAME}_test
    ${PROJECT_NAME}
  )

  # Generate test coverage report -> ${PROJECT_BINARY_DIR}/cmake_code_coverage/${PROJECT_NAME}/index.html
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
      TEST_BUILD_TARGETS ${PROJECT_NAME}_test
    )
  endif(cmake_code_coverage_FOUND)
endif()

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
