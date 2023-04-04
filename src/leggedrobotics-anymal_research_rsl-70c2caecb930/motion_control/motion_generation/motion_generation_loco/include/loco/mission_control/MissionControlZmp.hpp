/*
 * MissionControlZmp.hpp
 *
 *  Created on: Aug 22, 2014
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include "loco/mission_control/MissionControlBase.hpp"
#include "loco/mission_control/MissionControlSpeedFilter.hpp"
#include "loco/mission_control/MissionControlParameterHandler.hpp"
#include "loco/common/WholeBody.hpp"

// motion generation
#include "motion_generation/ContactScheduleZmp.hpp"

// robot_utils
#include <robot_utils/geometry/geometry.hpp>

// std utils
#include "std_utils/std_utils.hpp"

// std
#include <condition_variable>

class TiXmlHandle;

namespace loco {

using robot_utils::geometry::Tetragon;
using VertexList = robot_utils::geometry::Polygon::VertexList;
using VertexOrder = robot_utils::geometry::VertexOrder;

class MissionControlZmp : public MissionControlBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MissionControlZmp(Twist* desRobotTwist,
                    Pose* desRobotPose,
                    WholeBody& wholeBody,
                    ContactScheduleZmp& contactSchedule);
  ~MissionControlZmp() override = default;

  bool initialize(double dt) override;
  bool advance(double dt) override;
  bool loadParameters(const TiXmlHandle& handle) override;
  const Twist& getDesiredBaseTwistInControlFrame() const override;

  const Twist& getMaximumBaseTwistInControlFrame() const override;
  const Pose& getMinimalPoseOffset() const override;
  const Pose& getMaximalPoseOffset() const override;

  Twist getMaximumBaseTwistInControlFrameOfGait(const std::string& gaitName) const;

  CONSECUTIVE_ENUM(LocomotionMode, ModeStanding, ModeWalking, ModeStopping)
  const std::map<LocomotionMode, std::string> modeNameMap_ =
      {{LocomotionMode::ModeStanding, "stand"},
       {LocomotionMode::ModeWalking, "walk"},
       {LocomotionMode::ModeStopping, "stop"},
       {LocomotionMode::SIZE, "SIZE"}};

  // In walk mode, velocities are mapped to footholds positions and switch between standing and walking
  // In stand mode, pose offsets are mapped to body orientation angles. Legs do not swing.
  // In stop mode, nothing is commanded to the robot. It stays standing.
  void setTargetLocomotionMode(const LocomotionMode& locomotionMode);
  bool waitForLocomotionModeSwitch(const LocomotionMode& desiredLocomotionMode, const std::chrono::duration<int, std::milli>& timeout);

  const LocomotionMode& getLocomotionMode() const;
  const contact_schedule::ContactScheduleSwitchStatus& getLocomotionModeSwitchStatus() const;

  const std::string& getLocomotionModeName() const;

  bool isStopped() const noexcept;

  //! Add variables to the signal logger.
  bool addVariablesToLog(const std::string& ns = "/loco/MissionControlZmp") const override;

 protected:
  CONSECUTIVE_ENUM(LocomotionState, Standing, WalkingForwards, WalkingBackwards, SwitchToStanceFromForwards, SwitchToStanceFromBackwards, SwitchToWalkingForwards, SwitchToWalkingBackwards, ForceStance, ForceWalk, SwitchToStanceFromForceWalk)
  std::map<LocomotionState, std::string> LocomotionStateMap_ =
      {{LocomotionState::Standing, "Standing"},
       {LocomotionState::WalkingForwards, "WalkingForwards"},
       {LocomotionState::WalkingBackwards, "WalkingBackwards"},
       {LocomotionState::SwitchToStanceFromForwards, "SwitchToStanceFromForwards"},
       {LocomotionState::SwitchToStanceFromBackwards, "SwitchToStanceFromBackwards"},
       {LocomotionState::SwitchToWalkingForwards, "SwitchToWalkingForwards"},
       {LocomotionState::SwitchToWalkingBackwards, "SwitchToWalkingBackwards"},
       {LocomotionState::ForceStance, "ForceStance"},
       {LocomotionState::ForceWalk, "ForceWalk"},
       {LocomotionState::SwitchToStanceFromForceWalk, "SwitchToStanceFromForceWalk"},
       {LocomotionState::SIZE, "SIZE"}};

  CONSECUTIVE_ENUM(VelocityClippingType, Box, Ellipsoid)

  void enforceWithinBoundaries(Position& positionOffset) const;
  void enforceWithinBoundaries(EulerAnglesZyx& orientationOffset) const;
  void enforceWithinBoundaries(LinearVelocity& desiredLinearVelocity, VelocityClippingType velocityClippingType);
  void enforceWithinBoundaries(LocalAngularVelocity& desiredAngularVelocity, VelocityClippingType velocityClippingType) const;

  double interpolateJoystickAxis(double value, double minValue, double maxValue);

  bool updateLocomotionState();
  bool triggerSwitches();
  template <typename Derived>
  bool triggerSwitchToWalk(const Eigen::MatrixBase<Derived>& triggerSignals);
  bool triggerSwitchToForceWalk(double shrinkPolygonFactor);
  template <typename Derived>
  bool triggerSwitchToStand(const Eigen::MatrixBase<Derived>& triggerSignals, bool useGaitPatternFw);
  bool triggerSwitchWalkingDirection(bool walkingForward);
  bool triggerSwitchForceWalkToStand();

  //! Notify locomotion mode switch to callback.
  void setLocomotionMode(const LocomotionMode& locomotionMode);

  //! Establish switch to stand.
  bool switchToStand(contact_schedule::ContactScheduleSwitchStatus& status);

  //! Establish switch to walk.
  bool switchWalk(bool useGaitPatternFw, bool startGaitForBalancing = false);

  //! Establish a change of direction while walking.
  bool switchWalkingDirection(contact_schedule::ContactScheduleSwitchStatus& status, bool useGaitPatternFw);

  //! Set unfiltered base commands in control frame in the speed filter.
  virtual bool setUnfilteredDesiredBaseCommandsInControlFrame(double dt);

  //! Set unfiltered base twist in control frame in the speed filter.
  bool setUnfilteredDesiredBaseTwistInControlFrame();

  //! Update gait depending parameters.
  bool updateParameters(const std::string& gaitName);

  //! Updates gait depending parameters after a gait switch.
  virtual bool triggerParameterSwitch();

  //! Computes the largest rectangle that can be inscribed within the support polygon in base frame.
  static Tetragon getLargestInscribingRectangle(VertexList& tetragon);

  /*! Adds offset to maximum velocity when robot goes up a slope and reduces maximum velocity when going down a slope.
   *
   * @param maximumVelocity     The torso velocity limits for that gait.
   * @return                    Linear velocity offset for the observed slope.
   */
  LinearVelocity getMaximumVelocityOffsetOnSlope(const Eigen::Vector3d& maximumVelocity) const;

  //! A reference to the whole body.
  WholeBody& wholeBody_;

  //! A reference to the gait pattern.
  ContactScheduleZmp& contactSchedule_;

  Twist* desRobotTwist_;
   Pose* desRobotPose_;
   MissionControlSpeedFilter speedFilter_;

   //! Locomotion state: forward motion, backward motion, standing.
   LocomotionState locomotionState_;

   //! Locomotion mode: walking (includes forward motion, backward motion, standing), standing (we cannot switch).
   LocomotionMode locomotionMode_;

   //! Target locomotion mode when switching.
   LocomotionMode targetLocomotionMode_;

   //! Status of locomotion mode switch.
   contact_schedule::ContactScheduleSwitchStatus modeSwitchStatus_;

   //! Condition variable to switch locomotion modes.
   std::condition_variable modeSwitchCondVar_;

   //! Mutex for locomotion mode switch.
   std::mutex modeSwitchMutex_;

    //! Timer to trigger switches.
   std_utils::HighResolutionClockTimer chronoTimer_;

   //! Pin for switching to standing/walk or directions.
   bool didPin_;

   //! True if robot is switching to stand.
   bool isSwitchingToStand_;

   //! If unfiltered reference velocity reaches upper threshold, the timer is started to switch to stand.
   double switchToStandVelRefThresholdStart_;

   //! If unfiltered reference velocity reaches lower threshold, the switch to stand is aborted.
   double switchToStandVelRefThresholdEnd_;

   //! Time threshold for switch to walk/stand.
   double timeThresholdSwitchToStand_;

   //! True if robot is switching directions.
   bool isSwitchingDirection_;

   //! If unfiltered reference velocity reaches upper threshold, the timer is started to switch direction.
   double switchDirectionVelRefThresholdStart_;

   //! If unfiltered reference velocity reaches lower threshold, the switch of direction is aborded.
   double switchDirectionVelRefThresholdEnd_;

   //! Time threshold for switch direction.
   double timeThresholdSwitchDirection_;

   //! After the gait is forced to stance, it has to remain in that state for the following amount of time.
   double timeThresholdForceStance_;

   //! If timer exceeds this time threshold, the gait switcher is called (switch from force-walk to stand).
   double timeThresholdVelocityError_;

  //! If the robot is in stance and the velocity error is greater than this threshold, the gait is started in force-walk mode.
  double velocityErrorThresholdStart_;

  //! If the gait was forced to walk, it will go back to stance after the velocity error reaches this threshold.
  double velocityErrorThresholdEnd_;

  //! Factor by which the rectangle inscribed within the support polygon is shrunk.
  double shrinkPolygonFactor_;

  //! Gait depending parameters.
  mission_control_zmp::MissionControlParameterHandler params_;

  //! Previous active gait index (used to detect a gait switch).
  unsigned int previousActiveGaitId_;

  //! Previous desired gait index (used to detect a gait switch).
  unsigned int previousDesiredGaitId_;

  //! Filtered linear velocity of base in control frame.
  basic_filters::FirstOrderFilterKindrLinearVelocity filteredBaseLinearVelocitiesInControlFrame_;

  //! Determines how to clip the commanded velocities to a specified range (in a box or in an ellipsoid).
  VelocityClippingType velocityClippingType_;

  //! If true, the maximum possible speed while going up in increases and going down is decreased.
  bool enableVelocityOffsetsOnSlopes_;

  /*! The velocity offset per degree of incline angle.
   *  <velocity offset on slope> = <max velocity> * <slope in deg> * velocityOffsetMultiplierForSlopesPerDeg_;
   */
  Eigen::Vector2d velocityOffsetMultiplierForSlopesPerDeg_;

  //! The velocity offset for slopes. Should be zero for horizontal terrains.
  basic_filters::FirstOrderFilterKindrLinearVelocity linearVelocityOffsetOnSlope_;
};

} /* namespace loco */
