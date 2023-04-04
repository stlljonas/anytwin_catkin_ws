/*
 * SwingTrajectoryGeneratorSpline.hpp
 *
 *  Created on: May 30, 2016
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

// loco
#include <loco/common/TerrainModelBase.hpp>
#include <loco/common/WholeBody.hpp>
#include <loco/swing_trajectory_generation/SwingTrajectoryGeneratorBase.hpp>

// curbes
#include "curves/PolynomialSplineScalarCurve.hpp"

// robot utils
#include <robot_utils/function_approximators/polyharmonicSplines/BoundedRBF1D.hpp>

namespace loco {

//! Swing Trajectory Generator
/*!
 *  Follows a spline from current position along the desired trajectory to the foothold.
 */
class SwingTrajectoryGeneratorSpline : public SwingTrajectoryGeneratorBase {
 public:
  using Spline = curves::PolynomialSplineQuinticScalarCurve;
  using SplineCurve = std::vector<Spline>;

 public:
  SwingTrajectoryGeneratorSpline(WholeBody& wholeBody, TerrainModelBase& terrain);
  ~SwingTrajectoryGeneratorSpline() override = default;

  bool loadParameters(const TiXmlHandle& handle) override;
  bool getDesiredFootState(Position& positionWorldToDesiredFootInWorldFrame, LinearVelocity& linearVelocityDesiredFootInWorldFrame,
                           LinearAcceleration& linearAccelerationDesiredFootInWorldFrame,
                           const Position& positionWorldToDesiredFootholdInControlFrame, LegBase* leg, double dt) override;

  const SplineCurve& getSplineX() const;
  const SplineCurve& getSplineY() const;
  const SplineCurve& getSplineZ() const;

  bool compute(const Position& positionWorldToDesiredFootholdInWorldFrame, const Position& positionWorldToFootAtLiftOffInWorldFrame,
               unsigned int legId, double plannedSwingDuration);

 protected:
  double getInterpolationPhase(const LegBase& leg);
  double getEffectiveSwingDuration(const LegBase& leg);

  virtual void fitSpline(Spline& curve, const std::vector<double>& times, const std::vector<double>& values, const double initialVelocity,
                         const double initialAcceleration, const double finalVelocity, const double finalAcceleration);

 protected:
  WholeBody& wholeBody_;
  TerrainModelBase& terrain_;
  std::vector<double> swingHeightScaledTimes_;
  std::vector<double> swingHeightValues_;

  double nominalHeightOverStep_;
  double heightTrajectorySpeedInitial_;
  double heightTrajectorySpeedFinal_;

  SplineCurve curveX_;
  SplineCurve curveY_;
  SplineCurve curveZ_;
};

} /* namespace loco */
