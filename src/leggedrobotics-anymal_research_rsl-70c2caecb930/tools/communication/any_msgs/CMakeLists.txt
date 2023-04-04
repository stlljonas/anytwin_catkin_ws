# Minimum cmake version
cmake_minimum_required(VERSION 3.5.1)

# Create project
project(any_msgs)

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    sensor_msgs
    std_msgs
)

# Declare the message and service files to be built
add_message_files(
  FILES
    BoolStamped.msg
    Event.msg
    State.msg
    UserInteractionOption.msg
    ExtendedJointState.msg
    Float64Stamped.msg
    ImuWithTrigger.msg
    PointContact.msg
    SensorTimeInfo.msg
)
add_service_files(
  FILES
    ProcessFile.srv
    SetFloat32.srv
    SetFloat64.srv
    SetPose.srv
    SetTopic.srv
    SetUInt32.srv
    Toggle.srv
    UserInteraction.srv
    Shutdown.srv
    SetString.srv
    GetString.srv
    GetStrings.srv
)
generate_messages(
  DEPENDENCIES
    sensor_msgs
    std_msgs
)

# Generate a package
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    sensor_msgs
    std_msgs
)
