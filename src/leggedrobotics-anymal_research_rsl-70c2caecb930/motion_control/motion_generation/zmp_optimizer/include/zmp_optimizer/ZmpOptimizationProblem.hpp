/*
 * ZmpOptimizationProblem.hpp
 *
 *  Created on: 09.05, 2017
 *      Author: Fabian Jenelten
 */

#pragma once

// numerical optimization
#include "numopt_common/ConstrainedNonlinearProblem.hpp"
#include "numopt_common/numopt_common.hpp"

// zmp optimizer
#include "zmp_optimizer/NonLinearZmpFunctionConstraints.hpp"
#include "zmp_optimizer/NonLinearZmpObjectiveFunction.hpp"
#include "zmp_optimizer/zmp_optimizer.hpp"

namespace zmp {

class ZmpOptimizationProblem : public numopt_common::ConstrainedNonlinearProblem {
 public:
  explicit ZmpOptimizationProblem(ZmpOptimizerObjectiveHandler& objectiveHandler, double hessianRegularizer,
                                  const std_utils::EnumArray<zmp::CogDim, double>& finalMaxState,
                                  const std_utils::EnumArray<Ineq, bool>& enableInequalityConstraints, bool useConstraintHessian)
      : ConstrainedNonlinearProblem(std::shared_ptr<numopt_common::NonlinearObjectiveFunction>(
                                        new NonLinearZmpObjectiveFunction(objectiveHandler, hessianRegularizer)),
                                    std::shared_ptr<numopt_common::NonlinearFunctionConstraints>(new NonLinearZmpFunctionConstraints(
                                        objectiveHandler, finalMaxState, enableInequalityConstraints, useConstraintHessian))) {}

  ~ZmpOptimizationProblem() override = default;

  /* Consider the optimization proble
   *     min_{x} f(x)
   *       s.t.   c_in(x) >= 0
   *               c_eq(x) =  0
   * The Lagragian is given as
   *    L = f(x) + lambda *c_in(x) + mu*c_eq(x)
   * For our problem we have:
   *    > c_eq is linear in x, and
   *    > the mu'*Laplace(c_eq)*mu = 0
   * Thus, the hessian of Lagragian is
   *    Laplace(L) = Laplace(f) = H
   */

  virtual bool getLocalHessianOfLagrangian(numopt_common::SparseMatrix& hessian,
                                           const numopt_common::Parameterization& params) const override {
    if (static_cast<NonLinearZmpFunctionConstraints*>(constraints_.get())->useConstraintHessian()) {
      return numopt_common::ConstrainedNonlinearProblem::getLocalHessianOfLagrangian(hessian, params);
    }
    return objective_.get()->getGlobalHessian(hessian, params, true);
  }

  virtual bool getLowerLocalHessianOfLagrangian(numopt_common::SparseMatrix& hessian, const numopt_common::Parameterization& params,
                                                numopt_common::Scalar obj_factor, const numopt_common::Vector& lambda) const {
    if (static_cast<NonLinearZmpFunctionConstraints*>(constraints_.get())->useConstraintHessian()) {
      return numopt_common::ConstrainedNonlinearProblem::getLowerLocalHessianOfLagrangian(hessian, params, obj_factor, lambda);
    }

    numopt_common::SparseMatrix hessianFull;
    if (!objective_.get()->getGlobalHessian(hessianFull, params, true)) {
      return false;
    }
    hessian = obj_factor * hessianFull.triangularView<Eigen::Lower>();
    return true;
  }

  //! Initialize parameters and perform pre-computations
  bool initializeParameters(const zmp::ZmpInfo& zmpInfo, const MotionPlan* const motionPlan) {
    if (!static_cast<NonLinearZmpObjectiveFunction*>(objective_.get())->initialize(zmpInfo, motionPlan)) {
      MELO_WARN_STREAM("[ZmpOptimizationProblem::initializeParameters] Failed to initialize objective function.");
      return false;
    }

    if (!static_cast<NonLinearZmpFunctionConstraints*>(constraints_.get())->initialize(zmpInfo, motionPlan)) {
      MELO_WARN_STREAM("[ZmpOptimizationProblem::initializeParameters] Failed to initialize function constraints.");
      return false;
    }

    return true;
  }

  //! Call this function whenever params changes (i.e., before a new a new iteration starts).
  bool registerOptimizationStepInitCallback(const numopt_common::Parameterization& params) {
    return (static_cast<NonLinearZmpFunctionConstraints*>(constraints_.get())->doComputationBeforeIteration(params) &&
            static_cast<NonLinearZmpObjectiveFunction*>(objective_.get())->doComputationBeforeIteration(params)

    );
  }
};

}  // namespace zmp
