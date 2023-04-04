/*
 * SwingTrajectoryOptimizer.hpp
 *
 *  Created on: Jan. 15, 2017
 *      Author: Fabian Jenelten
 */
#pragma once

// swing trajectory generation
#include "swing_trajectory_generation/MotionPlan.hpp"

// motion generation utils
#include "motion_generation_utils/TaskHandlerBase.hpp"

// numerical optimization
#include <numopt_quadprog/ActiveSetFunctionMinimizer.hpp>
#include <numopt_common/numopt_common.hpp>
#include <numopt_common/QuadraticProblem.hpp>
#include <numopt_common/ParameterizationIdentity.hpp>

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// loco
#include "loco/common/typedefs.hpp"

namespace loco {

class SwingTrajectoryOptimizer {
 public:
  SwingTrajectoryOptimizer();
  virtual ~SwingTrajectoryOptimizer() = default;

  bool loadParameters(const TiXmlHandle& SwingTrajectoryGenerator);

  bool initialize();

  //! Runs the optimization
  bool computeTrajectory(sto::MotionPlan& motionPlan);

 protected:

  //! Pre-compute number of constraints and unknowns.
  bool setupConstraintProperties();

  //! Objective for tracking knot points.
  bool addContinuousTimeTrackingObjectives(
      unsigned int splineId,
      unsigned int coeffId,
      zmp::Derivative derivative,
      zmp::CogDim dim);

  //! Objective for approaching solution to the previous solution.
  bool addDiscreteTimeTrackingObjectives(
      double splineTimePreviousSolution,
      double sampleTime,
      unsigned int coeffId,
      zmp::Derivative derivative,
      zmp::CogDim dim);

  //! Objective for minimizing acceleration along trajectory.
  bool addAccelerationObjective(
      unsigned int splineId,
      unsigned int coeffId,
      zmp::CogDim dim);

  //! Equality constraints for initial and final conditions.
  bool addHardInitialAndFinalConstraints(
      unsigned int splineId,
      unsigned int coeffId,
      zmp::Derivative derivative,
      zmp::CogDim dim);

  //! Equality constraints for junction conditions (smooth transition up to second derivative between any two adjacent splines).
  bool addJunctionConstraints(
      unsigned int splineId,
      unsigned int coeffId,
      unsigned int nextCoeffId,
      zmp::Derivative derivative);

  //! Add hard (equality and inequality constraints) as well as soft (objective) tasks and construct QP matrices.
  bool addTasks();

  //! Convert solution coefficients to spline representation.
  bool createSolutionSplines();

  //! Hessian of a QP
  Eigen::MatrixXd costFunctionHessian_;

  //! Linear part of a QP.
  Eigen::VectorXd costFunctionLinearTerm_;

  //! Equality constraint Jacobian of a QP.
  Eigen::MatrixXd equalityConstraintJacobian_;

  //! Equality constraint target values of a QP.
  Eigen::VectorXd equalityConstraintTargetValues_;

  //! Equality constraint Jacobian of a QP.
  Eigen::MatrixXd inequalityConstraintJacobian_;

  //! Equality constraint target values of a QP.
  Eigen::VectorXd inequalityConstraintMinValues_;

  //! Index for equality constraints.
  unsigned int eqConstraintIdx_;

  //! Number of total equality constraints.
  unsigned int numEqConstraints_;

  //! Index for inequality constraints.
  unsigned int ineqConstraintIdx_;

  //! Number of discretization points for a full trajectory (from stance to stance).
  unsigned int numOfTotSwingSamples_;

  //! Number of unknowns.
  unsigned int solutionDimension_;

  //! Solution of the QP.
  numopt_common::ParameterizationIdentity solutionCoeffs_;

  //! Copy of the motion plan.
  sto::MotionPlan* motionPlan_;

  //! Optimization forsmulation for a QP (contains constraints and objectives).
  std::unique_ptr<numopt_common::QuadraticProblem> optimizationProblemQP_;

  //! QP solver.
  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver_;

  //! Object that handles soft and hard tasks.
  motion_gen::TaskHandlerBase taskHandler_;

  //! Regularization term for the Hessian (to render it positive definite).
  double hessianRegularizer_;

  //! If true, initial acceleration conditions are enforced by equality constraints.
  // Otherwise they are not set.
  bool setHardInitAccel_;

  //! Weight for minimizing the acceleration.
  double weightMinAccel_;

  //! Weight for tracking the knot positions (height trajectory only).
  double weightTrackingKnotHeightPos_;

  //! Weight for tracking the previous solution.
  std_utils::EnumArray<zmp::Derivative, double> weightPreviousSolution_;

  //! Weight for tracking desired swing height.
  double weightSwingHeight_;

  //! Knot position points for the trajectory height.
  std::vector<double> desiredKnotHeightOverPlane_;

  //! Knot points for the time, scaled to the interval (0,1) (from stance to stance).
  std::vector<double> desiredKnotTime_;

  //! Knot points for the time (from stance to stance).
  std::vector<double> desiredKnotTimeScaled_;

  //! Vector containing discrete-time instances of the sampled time domain.
  std::vector<std::vector<double>> timeInstantsPerSplineId_;

  //! Each spline is discretized with the following minimum number of sample points.
  unsigned int minNumOfSamplesPerSpline_;

  //! Desired height for the swing trajectory above ground.
  double desiredSwingHeightAbovePlane_;
};

} /* namespace sto */
