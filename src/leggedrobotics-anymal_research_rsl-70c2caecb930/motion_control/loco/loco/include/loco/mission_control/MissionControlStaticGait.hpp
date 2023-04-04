/*
 * MissionControlStaticGait.hpp
 *
 *  Created on: Aug 22, 2014
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include "loco/locomotion_controller/LocomotionController.hpp"
#include "loco/mission_control/MissionControlBase.hpp"
#include "loco/mission_control/MissionControlSpeedFilter.hpp"

// std_utils
#include "std_utils/std_utils.hpp"

namespace loco {

class MissionControlStaticGait : public MissionControlBase {
 public:
  MissionControlStaticGait(loco::Twist* desRobotTwist, loco::Pose* desRobotPose, LocomotionController* locomotionController);
  ~MissionControlStaticGait() override = default;

  bool initialize(double dt) override;
  bool advance(double dt) override;
  bool loadParameters(const TiXmlHandle& handle) override;
  const Twist& getDesiredBaseTwistInControlFrame() const override;

  const Twist& getMaximumBaseTwistInControlFrame() const override;
  const Pose& getMinimalPoseOffset() const override;
  const Pose& getMaximalPoseOffset() const override;

  // In walk mode, velocities are mapped to footholds positions and switch between standing and walking
  bool switchToWalkMode();
  // In stand mode, pose offsets are mapped to body orientation angles. Legs do not swing.
  bool switchToStandMode();

  const std::string& getLocomotionMode() const;

 private:
  enum LocomotionState { Standing = 0, WalkingForwards, WalkingBackwards };

  enum LocomotionMode { ModeStanding = 0, ModeWalking };

  double interpolateJoystickAxis(double value, double minValue, double maxValue);

  loco::Twist* desRobotTwist_;
  loco::Pose* desRobotPose_;
  MissionControlSpeedFilter speedFilter_;
  LocomotionController* locomotionController_;

  LocomotionState locomotionState_;
  LocomotionMode locomotionMode_;

  std_utils::HighResolutionClockTimer chronoTimer_;
  std_utils::HighResolutionClockTimer chronoTimerHeading_;

  bool standingDidPinTime_;

  bool walkingFwDidPinTime_;
  bool walkingFwDidPinTimeHeading_;
  bool walkingFwShouldGoToStand_;

  bool walkingBwDidPinTime_;
  bool walkingBwDidPinTimeHeading_;
  bool walkingBwShouldGoToStand_;

  double velocityThreshold_;
  double velocityHeadingThreshold_;
  double timeThresholdFwToBw_;
  double timeThresholdStandToWalk_;
  double timeThresholdWalkToStand_;

  // Turn gait pattern off
  bool switchToStand();

  // Turn gait pattern on (forward gait pattern)
  bool switchToForwardWalk(bool wasStanding = true);

  // Turn gait pattern on (backward gait pattern)
  bool switchToBackwardWalk(bool wasStanding = true);

  loco::Vector positionOffset_;

  std::map<LocomotionMode, std::string> modeNameMap_;
};

} /* namespace loco */
