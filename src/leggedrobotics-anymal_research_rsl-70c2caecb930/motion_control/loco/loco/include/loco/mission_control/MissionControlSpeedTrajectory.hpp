/*
 * MissionControlSpeedTrajectory.hpp
 *
 *  Created on: Apr 4, 2014
 *      Author: Christian Gehring
 */

#pragma once

// loco
#include "loco/mission_control/MissionControlBase.hpp"

// robot utils
#include "robot_utils/function_approximators/catmullRomSplines/CatmullRomSpline.hpp"

namespace loco {

class MissionControlSpeedTrajectory : public MissionControlBase {
 public:
  MissionControlSpeedTrajectory();
  ~MissionControlSpeedTrajectory() override = default;

  const Twist& getDesiredBaseTwistInControlFrame() const override;
  bool initialize(double dt) override;
  bool advance(double dt) override;
  bool loadParameters(const TiXmlHandle& handle) override;

  const Twist& getMaximumBaseTwistInControlFrame() const override;
  const Pose& getMinimalPoseOffset() const override;
  const Pose& getMaximalPoseOffset() const override;

 protected:
  double time_;
  robot_utils::catmull_rom::TrajectoryLinearVelocity linearVelocityTrajectory_;
  robot_utils::catmull_rom::TrajectoryLocalAngularVelocity localAngularVelocityTrajectory_;
  Twist currentBaseTwistInControlFrame_;
  bool isInterpolatingTime_;
  double cycleDuration_;
};

} /* namespace loco */
