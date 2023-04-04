/*
 * TrajectoryStateHandlerBase.hpp
 *
 *  Created on: 05.08, 2017
 *      Author: Fabian Jenelten
 */

#pragma once

//zmp
#include "motion_generation_utils/motion_generation.hpp"

// curves
#include "curves/polynomial_splines_containers.hpp"


namespace motion_generation {

class TrajectoryStateHandlerBase {
public:
  using MotionPlan = std_utils::EnumArray<zmp::CogDim, curves::PolynomialSplineContainerQuintic>;

  TrajectoryStateHandlerBase();
  virtual ~TrajectoryStateHandlerBase() = default;

  virtual bool initialize();

  //! True if one of the splines in the motion plan is empty.
  virtual bool isEmpty(bool verbose = false) const;

  //! Returns CoG state component at time.
  double getComStateInPlaneFrameAtTime(
        double tk,
        zmp::CogDim dim,
        zmp::Derivative derivative=zmp::Derivative::Zero) const;

  //! Returns CoG state component at final time.
  double getFinalComStateInPlaneFrame(
        zmp::CogDim dim,
        zmp::Derivative derivative=zmp::Derivative::Zero) const;

  //! Advance all splines in the motion plan.
  bool advance(double dt, bool verbose = true);

  //! Get global container duration.
  double getContainerDuration() const;

  //! Compute global container duration via spline durations.
  void updateContainerDuration();

  //! Set container duration externally (Note: this will not change the splines, so be sure when to use it).
  void setContainerDuration(double containerDuration);

  //! Get container time of spline.
  double getContainerTime() const;

  //! Read access to motion plan.
  const MotionPlan& getMotionPlan() const;

  //! Set motion plan equal to another one.
  void setMotionPlan(const MotionPlan& motionPlan);

  //! Set motion plan using a spline container.
  void setMotionPlan(const curves::PolynomialSplineContainerQuintic& splineContainer, zmp::CogDim dim);

  //! Set motion plan using a spline.
  bool setMotionPlan(const curves::PolynomialSplineQuintic& spline, zmp::CogDim dim);

  //! Set container time of all splines in the motion plan.
  bool setContainerTime(double containerTime);

  //! Reset spline.
  virtual bool reset(zmp::CogDim dim);
  virtual bool reset();

  //! Add splines.
  bool addSpline(const curves::PolynomialSplineQuintic& spline, zmp::CogDim dim);
  bool addSpline(curves::PolynomialSplineQuintic&& spline, zmp::CogDim dim);

  //! Check validity of state handler (use for debugging purposes only).
  bool checkTrajectoryStateHandler() const;

protected:

  //! Solution of the optimization is stored here.
  MotionPlan motionPlan_;

  //! actual valid time (valid for all splines).
  double containerTime_;

  //! Container duration (valid for all splines).
  double containerDuration_;

  // Optimization states (e.g. x,y,z,roll,pitch, yaw).
  std::vector<zmp::CogDim> optimizationState_;
};

} /* motion_generation */
