/*
 * NonLinearZmpFunctionConstraints.hpp
 *
 *  Created on: 09.05, 2017
 *      Author: Fabian Jenelten
 */

#pragma once

// zmp optimizer
#include "zmp_optimizer/LinearZmpFunctionConstraints.hpp"

namespace zmp {

class NonLinearZmpFunctionConstraints : public LinearZmpFunctionConstraints {
  /*
   * Implementation of the constraints
   *  c_in(x) >= 0
   *  c_eq(x) =  0
   */

 public:
  NonLinearZmpFunctionConstraints(ZmpOptimizerObjectiveHandler& objectiveHandler,
                                  const std_utils::EnumArray<zmp::CogDim, double>& finalMaxState,
                                  const std_utils::EnumArray<Ineq, bool>& enableInequalityConstraints, bool useConstraintHessian);

  ~NonLinearZmpFunctionConstraints() override = default;

  bool initialize(const ZmpInfo& zmpInfo, const MotionPlan* const motionPlan) override;

  //! Computation before iteration.
  bool doComputationBeforeIteration(const numopt_common::Parameterization& params);

  // Evaluate d gradient  c_in(x)/dx at x*.
  bool getLocalInequalityConstraintJacobian(numopt_common::SparseMatrix& jacobian, const numopt_common::Parameterization& params,
                                            bool newParams) override;

  // Evaluate Hessian dd c_in(x)/ddx at x*.
  bool getLocalInequalityConstraintHessian(numopt_common::SparseMatrix& hessian, const numopt_common::Parameterization& params,
                                           int iConstraint, bool newParams) override;

  // c_in(x) >= minValue.
  bool getInequalityConstraintMinValues(numopt_common::Vector& values) override;

  // c_in(x) <= maxValue.
  bool getInequalityConstraintMaxValues(numopt_common::Vector& values) override;

  // Evaluate c_in(x) at x*.
  bool getInequalityConstraintValues(numopt_common::Vector& values, const numopt_common::Parameterization& params, bool newParams) override;

  // Evaluate d c_eq(x)/dx at x*.
  bool getLocalEqualityConstraintJacobian(numopt_common::SparseMatrix& jacobian, const numopt_common::Parameterization& params,
                                          bool newParams) override;

  // Evaluate dd c_eq(x)/ddx at x*.
  bool getLocalEqualityConstraintHessian(numopt_common::SparseMatrix& hessian, const numopt_common::Parameterization& params,
                                         int iConstraint, bool newParams) override;

  //  c_eq(x) = targetValue.
  bool getEqualityConstraintTargetValues(numopt_common::Vector& values) override;

  // Evaluate c_eq(x) at x*.
  bool getEqualityConstraintValues(numopt_common::Vector& values, const numopt_common::Parameterization& params, bool newParams) override;

  // x >= minValue.
  bool getGlobalBoundConstraintMinValues(numopt_common::Vector& values) override;

  // x <= maxValue.
  bool getGlobalBoundConstraintMaxValues(numopt_common::Vector& values) override;

  // Maps the constraint index to zmp constraint.
  // If constraint index is not associated to a zmp constraint, this function returns -1.
  int mapToZmpConstaintIndex(unsigned int constraintIndex) const;

  bool useConstraintHessian() const noexcept;

 protected:
  bool addZmpConstraints(unsigned int coeffStartId, unsigned int splineId, double splineTime,
                         const robot_utils::geometry::Polygon::LineCoefficientList& lineCoefficients,
                         const Eigen::VectorXd& previousSplineCoefficients);

  bool computeConstraintValues(const numopt_common::Parameterization& params);

  //! Hessian of inequality constraint (zmp constraints only).
  std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> hessianZmpInequality_;

  //! Counter for nonlinear inequality constraints.
  unsigned int ineqConstraintIdxSQP_;

  //! Number of nonlinear inequality constraints.
  unsigned int numOfInequalityConstraintsSQP_;

  //! Number of linear inequality constraints.
  unsigned int numOfInequalityConstraintsQP_;

  //! Gradient of nonlinear inequality constraints.
  Eigen::MatrixXd InequalityGradientSQP_;

  //! Function evaluation of nonlinear inequality constraints.
  Eigen::VectorXd InequalityConstraintValuesSQP_;

  //! Equality constraint values at current iteration.
  numopt_common::Vector currentEqualityConstraintValues_;

  //! Inequality constraint values at current iteration.
  numopt_common::Vector currentInequalityConstraintValues_;

  //! If true, Hessian of constraints is provided.
  bool useConstraintHessian_;
};

}  // namespace zmp
