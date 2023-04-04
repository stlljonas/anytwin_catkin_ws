/*
 * SwingTrajectoryGeneratorLinearInterpolation.hpp
 *
 *  Created on: Feb 1, 2015
 *      Author: C. Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/swing_trajectory_generation/SwingTrajectoryGeneratorBase.hpp"

// robot utils
#include "robot_utils/function_approximators/catmullRomSplines/CatmullRomSpline.hpp"

// tinyxml
#include <tinyxml.h>

namespace loco {

//! Swing trajectory generator for dynamic gaits
/*! The desired position of the foot on the ground plane is obtained
 * by linearly interpolating the vector from hip to foot at lift-off
 * and the vector from current hip to desired foothold.
 * The foot height above the ground plane is obtained from a define trajectory.
 */
class SwingTrajectoryGeneratorLinearInterpolation : public SwingTrajectoryGeneratorBase {
 public:
  SwingTrajectoryGeneratorLinearInterpolation(WholeBody& wholeBody, TerrainModelBase& terrain);
  ~SwingTrajectoryGeneratorLinearInterpolation() override = default;

  bool getDesiredFootState(Position& positionWorldToDesiredFootInWorldFrame, LinearVelocity& linearVelocityDesiredFootInWorldFrame,
                           LinearAcceleration& linearAccelerationDesiredFootInWorldFrame,
                           const Position& positionWorldToDesiredFootholdInControlFrame, LegBase* leg, double dt) override;
  Position getPositionWorldToDesiredFootInWorldFrame(const Position& positionHipOnTerrainToDesiredFootholdInControlFrame, int legId);

  bool loadParameters(const TiXmlHandle& handle) override;

  friend std::ostream& operator<<(std::ostream& out, const SwingTrajectoryGeneratorLinearInterpolation& stGen);

  const robot_utils::catmull_rom::Trajectory1D& getStepInterpolationFunction() const;
  const robot_utils::catmull_rom::Trajectory1D& getSwingFootHeightTrajectory() const;

 protected:
  TorsoBase& torso_;
  Legs& legs_;
  TerrainModelBase& terrain_;

  robot_utils::catmull_rom::Trajectory1D stepInterpolationFunction_;
  robot_utils::catmull_rom::Trajectory1D swingFootHeightTrajectory_;

  Position getPositionWorldToHipOnPlaneAlongWorldZInWorldFrame(LegBase* leg);
  double getInterpolationPhase(const LegBase& leg);
  double getLateralComponentOfFootStep(double phase, double initialStepOffset, double stepGuess, LegBase* leg);
  double getHeadingComponentOfFootStep(double phase, double initialStepOffset, double stepGuess, LegBase* leg);
  Position getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(
      const LegBase& leg, const Position& positionHipOnTerrainToDesiredFootOnTerrainInControlFrame);
  Position getPositionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame(
      LegBase& leg, const Position& positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame);
};

} /* namespace loco */
