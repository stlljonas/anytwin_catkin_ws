# Project configuration
cmake_minimum_required(VERSION 3.5.1)
project(loco)

add_definitions(-std=c++11)
add_definitions(-DMELO_MIN_SEVERITY=MELO_SEVERITY_INFO)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Create a list of catkin package dependencies for both header and source files.
set(CATKIN_PACKAGE_DEPENDENCIES
  anymal_description
  basic_filters
  curves
  message_logger
  numopt_common
  parameter_handler
  robot_utils
  roco
  signal_logger
  std_utils
  tinyxml_tools
)

# Find catkin dependencies for building this package.
find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Find system dependencies providing a CMake module.
find_package(Eigen3 REQUIRED)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

###################################
## catkin specific configuration ##
###################################

# Declare this project as a catkin package
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    kindr
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/common/FootprintGenerator.cpp
  src/common/ParameterSet.cpp
  src/common/TerrainModelBase.cpp
  src/common/TerrainModelFreePlane.cpp
  src/common/TerrainModelHorizontalPlane.cpp
  src/common/WholeBody.cpp
  src/common/WholeBodyProperties.cpp
  src/common/WholeBodyStateDesired.cpp
  src/common/WholeBodyStateMeasured.cpp
  src/common/arms/ArmBase.cpp
  src/common/end_effectors/EndEffectorBase.cpp
  src/common/end_effectors/EndEffectorEnum.cpp
  src/common/end_effectors/EndEffectorProperties.cpp
  src/common/end_effectors/EndEffectorStateBase.cpp
  src/common/end_effectors/EndEffectorStateDesired.cpp
  src/common/end_effectors/EndEffectorStateMeasured.cpp
  src/common/end_effectors/FootBase.cpp
  src/common/end_effectors/Hand.cpp
  src/common/end_effectors/TimePoint.cpp
  src/common/end_effectors/Wheel.cpp
  src/common/end_effectors/WheelProperties.cpp
  src/common/joints/CylinderStates.cpp
  src/common/joints/DesiredJointStates.cpp
  src/common/joints/FingerStates.cpp
  src/common/joints/JointStates.cpp
  src/common/joints/MeasuredCylinderStates.cpp
  src/common/joints/MeasuredJointStates.cpp
  src/common/legs/ContactSchedule.cpp
  src/common/legs/LegBase.cpp
  src/common/legs/LegProperties.cpp
  src/common/legs/LegStateBase.cpp
  src/common/legs/LegStateLiftOff.cpp
  src/common/legs/LegStateTouchDown.cpp
  src/common/legs/LegsQuadruped.cpp
  src/common/limbs/HydraulicLimbStateMeasured.cpp
  src/common/limbs/LimbBase.cpp
  src/common/limbs/LimbLink.cpp
  src/common/limbs/LimbLinkGroup.cpp
  src/common/limbs/LimbProperties.cpp
  src/common/limbs/LimbStateDesired.cpp
  src/common/limbs/LimbStateMeasured.cpp
  src/common/limbs/LimbStrategy.cpp
  src/common/limbs/LimbStrategyBase.cpp
  src/common/torso/TorsoBase.cpp
  src/common/torso/TorsoProperties.cpp
  src/common/torso/TorsoStateDesired.cpp
  src/common/torso/TorsoStateMeasured.cpp
  src/common/torso/TorsoStateMeasuredInControlFrame.cpp
  src/contact_detection/ContactDetectorBase.cpp
  src/contact_detection/ContactDetectorChatteringCompensation.cpp
  src/contact_detection/ContactDetectorConstantDuringStance.cpp
  src/contact_detection/ContactDetectorFeedThrough.cpp
  src/contact_force_distribution/ContactForceDistribution.cpp
  src/contact_force_distribution/ContactForceDistributionBase.cpp
  src/contact_force_distribution/constraints/ActuatorEffortConstraint.cpp
  src/contact_force_distribution/constraints/FrictionConstraint.cpp
  src/contact_force_distribution/constraints/LimbLoadConstraint.cpp
  src/driving_control/SteeringControllerModule.cpp
  src/driving_control/SteeringModeModule.cpp
  src/event_detection/EventDetector.cpp
  src/event_detection/EventDetectorGaitPattern.cpp
  src/foot_placement_strategy/FootPlacementStrategyBase.cpp
  src/foot_placement_strategy/FootPlacementStrategyFreePlane.cpp
  src/foot_placement_strategy/FootPlacementStrategyStaticGait.cpp
  src/foothold_generation/FootholdGeneratorBase.cpp
  src/foothold_generation/FootholdGeneratorInvertedPendulum.cpp
  src/foothold_generation/FootholdGeneratorInvertedPendulumBase.cpp
  src/foothold_generation/FootholdGeneratorStaticGait.cpp
  src/gait_pattern/GaitPatternBase.cpp
  src/gait_pattern/GaitPatternFlightPhases.cpp
  src/gait_pattern/GaitPatternStaticGait.cpp
  src/heading_generation/HeadingGenerator.cpp
  src/limb_coordinator/LimbCoordinatorBase.cpp
  src/limb_coordinator/LimbCoordinatorDeprecated.cpp
  src/locomotion_controller/LocomotionController.cpp
  src/locomotion_controller/LocomotionControllerBase.cpp
  src/locomotion_controller/LocomotionControllerJump.cpp
  src/locomotion_controller/LocomotionControllerModules.cpp
  src/mission_control/MissionControlBase.cpp
  src/mission_control/MissionControlJoystick.cpp
  src/mission_control/MissionControlSpeedFilter.cpp
  src/mission_control/MissionControlSpeedTrajectory.cpp
  src/mission_control/MissionControlStaticGait.cpp
  src/motion_control/ContactInvariantDamper.cpp
  src/motion_control/JacobianTranspose.cpp
  src/motion_control/MotionControllerBase.cpp
  src/motion_control/VirtualModelController.cpp
  src/motion_control/VirtualModelControllerContactInvariantDamper.cpp
  src/planner/PlannerBase.cpp
  src/planner/PlannerBoostThread.cpp
  src/planner/PlannerWorker.cpp
  src/state_switcher/StateSwitcher.cpp
  src/swing_trajectory_generation/SwingTrajectoryGeneratorLinearInterpolation.cpp
  src/swing_trajectory_generation/SwingTrajectoryGeneratorSpline.cpp
  src/swing_trajectory_generation/SwingTrajectoryGeneratorStaticGait.cpp
  src/terrain_perception/TerrainPerceptionBase.cpp
  src/terrain_perception/TerrainPerceptionFreePlane.cpp
  src/terrain_perception/TerrainPerceptionFreePlaneAdapted.cpp
  src/terrain_perception/TerrainPerceptionHorizontalPlane.cpp
  src/torso_control/ComSupportControlBase.cpp
  src/torso_control/ComSupportControlDynamicGait.cpp
  src/torso_control/ComSupportControlLeverConfiguration.cpp
  src/torso_control/ComSupportControlStaticGait.cpp
  src/torso_control/TerrainAdapter.cpp
  src/torso_control/TorsoControlDynamicGaitFreePlane.cpp
  src/torso_control/TorsoControlGaitContainer.cpp
  src/torso_control/TorsoControlStaticGait.cpp
  src/torso_control/TorsoControlTest.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${kindr_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########
## Clang ##
###########

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
