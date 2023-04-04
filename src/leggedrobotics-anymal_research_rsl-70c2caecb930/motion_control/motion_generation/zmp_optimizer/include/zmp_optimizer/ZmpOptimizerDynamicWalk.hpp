/*
 * ZmpOptimizer.hpp
 *
 *  Created on: Nov 25, 2014
 *      Author: C. Dario Bellicoso
 */

#pragma once

// loco
#include <zmp_optimizer/ZmpOptimizerObjectiveHandler.hpp>
#include "zmp_optimizer/ZmpOptimizationProblem.hpp"
#include "zmp_optimizer/ZmpOptimizationProblemQP.hpp"
#include "zmp_optimizer/ZmpOptimizerBase.hpp"

// eigen
#include <Eigen/Core>
#include <Eigen/Sparse>

// numerical optimization
#include <numopt_common/numopt_common.hpp>
#include <numopt_quadprog/ActiveSetFunctionMinimizer.hpp>             // QP active set method
#include <numopt_sqp_line_search/SqpFunctionMinimizerLineSearch.hpp>  // SQP method using AS solver (nonlin solver)

// curves
#include "curves/polynomial_splines_containers.hpp"

// robot utils
#include <robot_utils/geometry/geometry.hpp>

// std utils
#include <std_utils/std_utils.hpp>

// stl
#include <memory>

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>

namespace loco {

class ZmpOptimizerDynamicWalk : public ZmpOptimizerBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ZmpOptimizerDynamicWalk();

  // todo: implement copy constructor to do proper memory management
  //  ZmpOptimizerDynamicWalk(const ZmpOptimizerDynamicWalk& optimizer);

  ~ZmpOptimizerDynamicWalk() override = default;

  virtual bool initialize(double wholeBodyMass, const Eigen::Matrix3d& torsoInertiaTensorInBaseFrame);

  virtual bool initialize() override;

  virtual bool initializeSolvers();

  bool loadParameters(const TiXmlHandle& handle) override;
  bool addToLogger() override;

  //! Run the optimization.
  bool computeTrajectory(zmp::MotionPlan& motionPlan) override;

  void stop() override;

 protected:
  using ZmpSplineCoeff = Eigen::Matrix<double, zmp::splineCoeffs, 1u>;

  //! Processes inputs.
  bool readMotionPlan(zmp::MotionPlan& motionPlan);

  //! Compute the solution space and the number of constraints.
  bool setupConstraintProperties();

  //! Find spline coefficients to initialize the SQP solver.
  bool initializeSplineCoefficients();

  //! Convert support polygons to spline representation.
  bool createSplineInfoSequence();

  //! Get size of support polygon for spline representation.
  virtual size_t getSizeOfSupportPolygonForSplineRepresentation();

  //! Solve the optimization problem (run the SQP).
  bool runOptimization();

  //! Extract the solution and create a motion plan.
  bool createSolutionSplines();

  //! Solution in coefficient representation.
  Eigen::VectorXd optimizedSolution_;

  //! Object that computes objective and constraint matrices.
  zmp::ZmpOptimizerObjectiveHandler objectiveHandler_;

  //! Dimensional space of the problem (x,y,z,theta ).
  std::vector<zmp::CogDim> optimizationDofs_;

  //! True if at least one constraint is nonlinear or one objective is non-quadratic.
  bool isNonlinearOptimization_;

  //! Optimization formulation for a QP (contains constraints and objectives).
  std::unique_ptr<zmp::ZmpOptimizationProblemQP> zmpOptimizationProblemQP_;

  //! Optimization formulation for a nonlinear program (contains constraints and objectives).
  std::unique_ptr<zmp::ZmpOptimizationProblem> zmpOptimizationProblem_;

  //! QP solver.
  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver_;

  //! SQP solver (nonlin solver).
  std::unique_ptr<numopt_sqp::SqpFunctionMinimizerLineSearch> sqpSolver_;

  //! Line Search options for SQP solver.
  numopt_sqp::LineSearchOptions lineSearchOpts_;

  //! Max number of SQP iteration.
  unsigned int maxIter_;

  //! For debugging purposes.
  bool verbose_;

  //! Regularizer term added to the Hessian Q s.t. Q>0.
  double hessianRegularizer_;

  //! Vector encoding the availability of different inequality constraints.
  std_utils::EnumArray<zmp::Ineq, bool> enableInequalityConstraints_;

  //! Boundary constraints for final point (position, velocity, acceleration).
  std_utils::EnumArray<zmp::CogDim, double> finalMaxStateBox_;

  std_utils::HighResolutionClockTimer timer_;

  //! Distance between the safe polygon and the support polygon segments.
  double delta_;

  //! An offset which determines how wide a line will be expanded into a rectangle when computing the support polygon sequence.
  double lineToRectangleOffset_;

  //! An offset which determines how wide a point will be expanded into a rectangle when computing the support polygon sequence.
  double pointToRectangleOffset_;

  //! Circle inequality constraints are transformed to rectangular constraints with this number of vertices.
  unsigned int numOfVertecesPerCircleConstraint_;

  //! Copy of the motion plan.
  zmp::MotionPlan* motionPlan_;

  //! If true, Hessian of constraints is incorporated in optimization.
  bool useConstraintHessian_;

  //! Number of previous unknown spline coefficients.
  unsigned int numOfPreviousCoeffs_;

  //! Number of actual unknown spline coefficients.
  unsigned int numOfCurrentCoeffs_;

  //! Struct containing objects used for the zmp-optimization.
  zmp::ZmpInfo zmpInfo_;

  //! Number of samples per spline.
  unsigned int numOfSamplesPerSpline_;

  //! Velocity boosting gains. For xy optimization only.
  // Spine is generated for z axis by connecting initial and final state. Initial velocity is boosted with a gain alpha*exp(-lambda*Delta
  // x^2), where Delta x is the position error between initial and final position.
  double expZAlpha_;
  double expZLambda_;
};

} /* namespace loco */
