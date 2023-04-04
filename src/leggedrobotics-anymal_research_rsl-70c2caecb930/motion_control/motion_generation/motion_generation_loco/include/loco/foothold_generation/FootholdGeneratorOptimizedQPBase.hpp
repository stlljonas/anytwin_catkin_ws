/*
 * FootholdGeneratorOptimizedQPBase.hpp
 *
 *  Created on: Mar 15, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// loco
#include "loco/foothold_generation/FootholdGeneratorBase.hpp"


// numerical optimization
#include "numopt_quadprog/ActiveSetFunctionMinimizer.hpp"
#include "numopt_common/numopt_common.hpp"
#include "numopt_common/QuadraticProblem.hpp"
#include "numopt_common/ParameterizationIdentity.hpp"

// eigen
#include <Eigen/Core>

// kindr
#include <kindr/Core>
#include "loco/foothold_generation/FootholdPlan.hpp"

class TiXmlHandle;

namespace loco {

class FootholdGeneratorOptimizedQPBase : public FootholdGeneratorBase {
 public:
  using Base = FootholdGeneratorBase;
  using Weight = Eigen::DiagonalMatrix<double, 2, 2>;
  using Position2d = Eigen::Vector2d;

  FootholdGeneratorOptimizedQPBase();
  ~FootholdGeneratorOptimizedQPBase() override = default;

  bool loadParameters(const TiXmlHandle& handle) override;
  virtual bool initialize(double dt) = 0;

  //! Run optimization.
  virtual bool compute(foothold_generator::FootholdPlan& plan) = 0;

  virtual void stop();

 protected:

  //! Initialize the optimization for leg with index legId.
  virtual bool setupConstraintProperties(unsigned int legId, const foothold_generator::FootholdPlan& plan) = 0;

  //! Compute Hessian Q and linear term l of the cost function x'Qx + l'*x.
  virtual bool setupCostFunction(unsigned int legId, const foothold_generator::FootholdPlan& plan) = 0;

  //! Compute Jacobian A_in and min value b_in of the linear ineqconstraints A_in*x>=b_in.
  virtual bool setupInequalityConstraints(unsigned int legId, const foothold_generator::FootholdPlan& plan) = 0;

  //! Compute Jacobian A_eq and target values b_in of the linear eqconstraints A_eq*x=b_eq.
  virtual bool setupEqualityConstraints(unsigned int legId, const foothold_generator::FootholdPlan& plan) = 0;

  /*! Add the equality constraint:
   *      Ax-b = 0
   *  as
   *      min 0.5 (x'A'WAx) - b'WAx
   *
   */
  virtual void addObjectiveForLeg(
      const Eigen::Matrix<double,2,2>& equalityJacobian,
      const Eigen::Vector2d& equalityTargetValues,
      const Weight& taskWeights);

  /*! Add an equality objective:
   *    \f[
   *    x = b
   *    \f]
   * to the cost function as:
   *    \f[
   *    \textrm{min.} \frac{1}{2} x' W x - b' W x
   *    \f]
   * where \f$W\f$ is a cost Hessian derived from taskWeights and \f$b\f$ is equalityTargetValues.
   *
   * @param equalityTargetValues Target value for the optimization variable.
   * @param taskWeights Weight of the term in the cost function.
   */
  void addApproachingObjective(
      const Eigen::Vector2d& equalityTargetValues,
      const Weight& taskWeights);

  //! Set up optimization problem as a QP.
  virtual bool setUpQP(unsigned int legId, const foothold_generator::FootholdPlan& plan) = 0;

  // QP problem.
  Eigen::MatrixXd costFunctionHessian_;
  Eigen::VectorXd costFunctionLinearTerm_;
  Eigen::MatrixXd equalityConstraintJacobian_;
  Eigen::VectorXd equalityConstraintTargetValues_;
  Eigen::MatrixXd inequalityConstraintJacobian_;
  Eigen::VectorXd inequalityConstraintMinValues_;
  numopt_common::ParameterizationIdentity solutionFootholdInPlaneFrame_;

  //! Optimization formulation for a QP (contains constraints and objectives).
  std::unique_ptr<numopt_common::QuadraticProblem> optimizationProblemQP_;

  //! QP solver
  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver_;

  //! State space.
  unsigned int solutionDimension_;

  //! Number of hard constrains.
  unsigned int numOfEqualityConstraints_;
  unsigned int numOfInequalityConstraints_;

  //! Counter of hard constraints.
  unsigned int equalityConstraintsCounter_;
  unsigned int inequalityConstraintsCounter_;

};

} /* namespace loco */
