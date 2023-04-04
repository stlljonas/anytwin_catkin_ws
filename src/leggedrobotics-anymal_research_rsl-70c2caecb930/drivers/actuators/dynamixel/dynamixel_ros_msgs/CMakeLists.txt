# Minimum cmake version
cmake_minimum_required(VERSION 3.5.1)

# Create project
project(dynamixel_ros_msgs)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Find catkin macros and libraries
find_package(catkin REQUIRED
	COMPONENTS
		message_generation
		std_msgs
)

# Declare the service and message files to be built
add_service_files(
  FILES
    GetAngleLimits.srv
    GetCompliance.srv
    GetGoalAcceleration.srv
    GetGoalPosition.srv
    GetGoalTorque.srv
    GetMaxTorque.srv
    GetMovingSpeed.srv
    GetPidGains.srv
    GetTorqueControlModeEnable.srv
    GetTorqueEnable.srv
    SetAngleLimits.srv
    SetCompliance.srv
    SetGoalAcceleration.srv
    SetGoalPosition.srv
    SetGoalTorque.srv
    SetMaxTorque.srv
    SetMovingSpeed.srv
    SetPidGains.srv
    SetTorqueControlModeEnable.srv
    SetTorqueEnable.srv
)

generate_messages(DEPENDENCIES std_msgs)

# Generate a package
catkin_package(
  CATKIN_DEPENDS
  	message_runtime
    std_msgs
)