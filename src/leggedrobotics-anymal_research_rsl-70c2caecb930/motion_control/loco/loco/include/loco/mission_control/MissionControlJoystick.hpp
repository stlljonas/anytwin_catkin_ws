/*
 * MissionControlJoystick.hpp
 *
 *  Created on: Mar 7, 2014
 *      Author: Christian Gehring
 */

#pragma once

// loco
#include "loco/mission_control/MissionControlBase.hpp"

// robot utils
#include "robot_utils/sensors/Joystick.hpp"

// basic filters
#include <basic_filters/FilteredVariable.hpp>

namespace loco {

class MissionControlJoystick : public MissionControlBase {
 public:
  explicit MissionControlJoystick(robot_utils::Joystick* joyStick);
  ~MissionControlJoystick() override = default;

  bool initialize(double dt) override;
  bool advance(double dt) override;
  const Twist& getDesiredBaseTwistInControlFrame() const override;
  const Position& getDesiredPositionMiddleOfFeetToBaseInWorldFrame() const;
  const RotationQuaternion& getDesiredOrientationControlToBase() const;

  const Twist& getMaximumBaseTwistInControlFrame() const override;
  const Pose& getMinimalPoseOffset() const override;
  const Pose& getMaximalPoseOffset() const override;

  bool loadParameters(const TiXmlHandle& handle) override;

  double interpolateJoystickAxis(double value, double minValue, double maxValue);

  friend std::ostream& operator<<(std::ostream& out, const MissionControlJoystick& joystick);

 protected:
  robot_utils::Joystick* joyStick_;
  Twist baseTwistInControlFrame_;
  Twist maximumBaseTwistInControlFrame_;
  Position desiredPositionMiddleOfFeetToBaseInWorldFrame_;
  Position minimalPositionMiddleOfFeetToBaseInWorldFrame_;
  Position maximalPositionMiddleOfFeetToBaseInWorldFrame_;
  RotationQuaternion desiredOrientationControlToBase_;
  EulerAnglesZyx minimalOrientationControlToBase_;
  EulerAnglesZyx maximalOrientationControlToBase_;
  Pose minimalPoseOffset_;
  Pose maximalPoseOffset_;

  //! filtered speeds [sagittal; coronal; turning]
  std::vector<basic_filters::FilteredDouble> filteredVelocities_;
};

} /* namespace loco */
