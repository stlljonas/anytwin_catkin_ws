# Minimum cmake version
cmake_minimum_required(VERSION 3.5.1)

# Create project
project(anymal_msgs)

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    any_msgs
    geometry_msgs
    message_generation
    sensor_msgs
    std_msgs
)

# Declare the message and service files to be built
add_message_files(
  FILES
    AnymalLowLevelControllerState.msg
    AnymalState.msg
    BMSState.msg
    Contact.msg
    Contacts.msg
    GaitPattern.msg
    GaitPatterns.msg
    LegAttributes.msg
)
add_service_files(
  FILES
    AnymalLowLevelControllerGoToState.srv
    GetActiveController.srv
    GetAvailableControllers.srv
    ResetStateEstimator.srv
    SetBaseHeight.srv
    SetBasePose.srv
    SetBoolean.srv
    SwitchController.srv
)
generate_messages(
  DEPENDENCIES
    any_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
)

# Generate a package
catkin_package(
  CATKIN_DEPENDS
    any_msgs
    geometry_msgs
    message_runtime
    sensor_msgs
    std_msgs
)
