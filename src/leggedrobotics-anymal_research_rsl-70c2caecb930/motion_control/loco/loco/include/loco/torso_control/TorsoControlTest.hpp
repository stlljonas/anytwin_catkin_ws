/*
 * TorsoControlTest.hpp
 *
 *  Created on: Oct 9, 2014
 *      Author: C. Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/WholeBody.hpp"
#include "loco/gait_pattern/GaitPatternStaticGait.hpp"
#include "loco/torso_control/ComSupportControlStaticGait.hpp"
#include "loco/torso_control/TorsoControlGaitContainer.hpp"

// robot utils
#include <robot_utils/schedule/ProfileLogChirpUpSweep.hpp>
#include <robot_utils/schedule/ProfileRamp.hpp>
#include <robot_utils/schedule/ProfileSinusoid.hpp>
#include <robot_utils/schedule/ProfileStep.hpp>
#include <robot_utils/schedule/Schedule.hpp>
#include <std_utils/std_utils.hpp>

namespace loco {

class TorsoControlTest : public TorsoControlGaitContainer {
 private:
  using Base = TorsoControlGaitContainer;

 public:
  TorsoControlTest(WholeBody& wholeBody, TerrainModelBase& terrain, GaitPatternStaticGait& gaitPattern,
                   ComSupportControlStaticGait& comSupportControl);
  ~TorsoControlTest() override = default;

  bool initialize(double dt) override;
  bool advance(double dt) override;

  bool loadParameters(const TiXmlHandle& handle) override;

  virtual void setIsInStandConfiguration(bool isInStandConfiguration);
  bool setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t) override;

  virtual bool addVariablesToLog(bool updateLogger);

  void setMainBodyDesiredHeightFromTerrain(double height);

 protected:
  double time_;

  loco::Vector positionTrackingError_;
  loco::Vector velocityTrackingError_;

  // for visualization
  loco::Vector positionWorldToDesiredBaseInWorldFrame_;
  loco::Vector linearVelocityDesiredBaseInWorldFrame_;

  robot_utils::Schedule<double> scheduleX_;
  robot_utils::Schedule<double> scheduleY_;
  robot_utils::Schedule<double> scheduleZ_;

  robot_utils::Schedule<double> scheduleYaw_;
  robot_utils::Schedule<double> schedulePitch_;
  robot_utils::Schedule<double> scheduleRoll_;

  parameter_handler::Parameter<double> paramTorsoHorizPosSinusoidFreqHz_;
  std_utils::SteadyClockTimer timer_;
};

} /* namespace loco */
