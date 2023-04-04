/*
 * SwingTrajectoryGeneratorStaticGait.hpp
 *
 *  Created on: Feb 6, 2015
 *      Author: C. Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/swing_trajectory_generation/SwingTrajectoryGeneratorBase.hpp"

// curves
#include "curves/PolynomialSplineScalarCurve.hpp"

namespace loco {

class SwingTrajectoryGeneratorStaticGait : public SwingTrajectoryGeneratorBase {
 public:
  SwingTrajectoryGeneratorStaticGait(WholeBody& wholeBody, TerrainModelBase& terrain);
  ~SwingTrajectoryGeneratorStaticGait() override = default;

  bool loadParameters(const TiXmlHandle& handle) override;

  bool getDesiredFootState(Position& positionWorldToDesiredFootInWorldFrame, LinearVelocity& linearVelocityDesiredFootInWorldFrame,
                           LinearAcceleration& linearAccelerationDesiredFootInWorldFrame,
                           const Position& positionWorldToDesiredFootholdInControlFrame, LegBase* leg, double dt) override;

 protected:
  Position getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(
      const LegBase& leg, const Position& positionFootOnTerrainAtLiftOffToDesiredFootOnTerrainInControlFrame);
  double getLateralComponentOfFootStep(double interpolationTime, double lateralTarget);
  double getHeadingComponentOfFootStep(double interpolationtime, double headingTarget);

  double getInterpolationPhaseForPlaneMotion(const LegBase& leg);
  double getInterpolationPhaseForHeightMotion(const LegBase& leg);

  double getInterpolationPhase(const LegBase& leg);

  WholeBody& wholeBody_;
  TerrainModelBase& terrain_;

  TorsoBase& torso_;
  Legs& legs_;

  curves::PolynomialSplineQuinticScalarCurve planeTrajectory_;
  curves::PolynomialSplineQuinticScalarCurve heightTrajectory_;

  double endPlaneMotionAtPhase_;
  double endHeightMotionAtPhase_;

  double planeMotionInterpolationParameter_;
  double heightMotionInterpolationParameter_;
};

} /* namespace loco */
