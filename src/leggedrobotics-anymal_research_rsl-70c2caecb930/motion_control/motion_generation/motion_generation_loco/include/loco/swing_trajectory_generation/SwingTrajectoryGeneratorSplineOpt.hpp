/*
 * SwingTrajectoryGeneratorSplineOpt.hpp
 *
 *  Created on: June 25, 2019
 *      Author: Fabian Jenelten
 */

#pragma once

 //loco
#include "loco/common/ModuleBase.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/common/TerrainModelBase.hpp"
#include "loco/swing_trajectory_generation/SwingTrajectoryGeneratorModule.hpp"
#include "loco/swing_trajectory_generation/SwingTrajectoryParameterHandler.hpp"
#include "loco/common/legs/Legs.hpp"
#include "loco/gait_pattern/contact_schedule_anymal.hpp"

// curves
#include "curves/PolynomialSplineScalarCurve.hpp"

// robot utils
#include <robot_utils/function_approximators/polyharmonicSplines/BoundedRBF1D.hpp>

// motion generation.
#include "motion_generation/ContactScheduleZmp.hpp"


namespace loco {


//! Swing Trajectory Generator
/*!
 *  Follows a spline from current position along the desired trajectory to the foothold.
 */
class SwingTrajectoryGeneratorSplineOpt : public SwingTrajectoryGeneratorModule  {
 public:
  using Spline = curves::PolynomialSplineQuinticScalarCurve;
  using SplineCurve = std::vector<Spline>;

  SwingTrajectoryGeneratorSplineOpt(
      WholeBody& wholeBody,
      TerrainModelBase& terrain,
      ContactScheduleZmp& contactSchedule);
  ~SwingTrajectoryGeneratorSplineOpt() override = default;

  bool loadParameters(const TiXmlHandle& handle) override;
  bool getDesiredFootState(
      Position& positionWorldToDesiredFootInWorldFrame,
      LinearVelocity& linearVelocityDesiredFootInWorldFrame,
      LinearAcceleration& linearAccelerationDesiredFootInWorldFrame,
      const Position& positionWorldToDesiredFootholdInControlFrame,
      LegBase* leg, double dt) override;

  const SplineCurve& getSplineX() const;
  const SplineCurve& getSplineY() const;
  const SplineCurve& getSplineZ() const;

  virtual bool compute(
      const Position& positionWorldToDesiredFootholdInWorldFrame,
      const Position& positionWorldToFootAtLiftOffInWorldFrame,
      unsigned int legId,
      double plannedSwingDuration);

  bool initialize(double dt) override;
  bool advance(double dt) override;

 protected:
  virtual void fitSpline(
      Spline& curve, const std::vector<double>& times, const std::vector<double>& values,
      const double initialVelocity, const double initialAcceleration,
      const double finalVelocity, const double finalAcceleration);

  //! If true, swing leg is kick forward.
  bool kickLegAhead(unsigned int legId) const;

  //! A reference to the whole body.
  WholeBody& wholeBody_;

  //! A reference to the terrain.
  TerrainModelBase& terrain_;

  //! A reference to the gait pattern.
  ContactScheduleZmp& contactSchedule_;

  //! Swing leg trajectory x-component.
  SplineCurve curveX_;

  //! Swing leg trajectory y-component.
  SplineCurve curveY_;

  //! Swing leg trajectory z-component.
  SplineCurve curveZ_;

  //! Gait depending swing trajectory parameters.
  swing_traj_opt::SwingTrajectoryParameterHandler swingParams_;
};

} /* namespace loco */
