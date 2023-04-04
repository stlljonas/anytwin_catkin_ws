cmake_minimum_required(VERSION 3.5.1)
project(anydrive_msgs)

add_definitions(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    sensor_msgs
    std_msgs
)

add_message_files(
  FILES
    CalibrationMode.msg
    CalibrationType.msg
    Command.msg
    Commands.msg
    DriveInfo.msg
    ErrorStateBehavior.msg
    FsmState.msg
    JointPositionConfiguration.msg
    JointPositionConfigurations.msg
    JointPositionLimits.msg
    Mode.msg
    PidGains.msg
    Reading.msg
    ReadingExtended.msg
    Readings.msg
    ReadingsExtended.msg
    State.msg
    StateExtended.msg
    States.msg
    StatesExtended.msg
)

add_service_files(
  FILES
    AchieveJointPositionConfiguration.srv
    Calibrate.srv
    EraseFlashStorage.srv
    GetCalibrationState.srv
    GetControlGains.srv
    GetDirection.srv
    GetDriveInfo.srv
    GetErrorStateBehavior.srv
    GetJointPositionLimits.srv
    GetMaxCurrent.srv
    GetMaxMotorVelocity.srv
    ResetCustomCalibrationsToFactory.srv
    ResetFlashStorageSections.srv
    SendControlword.srv
    SendSdoRead.srv
    SendSdoWrite.srv
    SetControlGains.srv
    SetDirection.srv
    SetDriveInfoString.srv
    SetDriveInfoUint16.srv
    SetErrorStateBehavior.srv
    SetFsmGoalState.srv
    SetJointPositionLimits.srv
    SetMaxCurrent.srv
    SetMaxMotorVelocity.srv
    WriteConfiguration.srv
    WriteFactoryCalibration.srv
)

generate_messages(
  DEPENDENCIES
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_generation
    sensor_msgs
    std_msgs
)
