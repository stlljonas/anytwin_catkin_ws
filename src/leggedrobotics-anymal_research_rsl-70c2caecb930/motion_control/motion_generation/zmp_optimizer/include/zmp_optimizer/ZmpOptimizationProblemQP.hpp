/*
 * ZmpOptimizationProblemQP.hpp
 *
 *  Created on: 09.05, 2017
 *      Author: Fabian Jenelten
 */

#pragma once

// numerical optimization
#include "numopt_common/QuadraticProblem.hpp"
#include "numopt_common/numopt_common.hpp"

// zmp optimizer
#include "zmp_optimizer/LinearZmpFunctionConstraints.hpp"
#include "zmp_optimizer/QuadraticZmpObjectiveFunction.hpp"
#include "zmp_optimizer/zmp_optimizer.hpp"

namespace zmp {

class ZmpOptimizationProblemQP : public numopt_common::QuadraticProblem {
 public:
  using QuadraticObjectiveFunctionPtr = std::shared_ptr<numopt_common::QuadraticObjectiveFunction>;
  using LinearFunctionConstraintsPtr = std::shared_ptr<numopt_common::LinearFunctionConstraints>;

 public:
  ZmpOptimizationProblemQP(ZmpOptimizerObjectiveHandler& objectiveHandler, double hessianRegularizer,
                           const std_utils::EnumArray<zmp::CogDim, double>& finalMaxState,
                           const std_utils::EnumArray<Ineq, bool>& enableInequalityConstraints, bool useConstraintHessian)
      : QuadraticProblem(QuadraticObjectiveFunctionPtr(new QuadraticZmpObjectiveFunction(objectiveHandler, hessianRegularizer)),
                         LinearFunctionConstraintsPtr(new LinearZmpFunctionConstraints(
                             objectiveHandler, finalMaxState, enableInequalityConstraints, useConstraintHessian))) {}

  ~ZmpOptimizationProblemQP() override = default;

  //! Initialize parameters and perform pre-computations.
  bool initializeParameters(const zmp::ZmpInfo& zmpInfo, const MotionPlan* const motionPlan) {
    if (!static_cast<QuadraticZmpObjectiveFunction*>(objective_.get())->initialize(zmpInfo, motionPlan)) {
      MELO_WARN_STREAM("[ZmpOptimizationProblemQP::initializeParameters] Failed to initialize quadratic objective function.");
      return false;
    }

    if (!static_cast<LinearZmpFunctionConstraints*>(constraints_.get())->initialize(zmpInfo, motionPlan)) {
      MELO_WARN_STREAM("[ZmpOptimizationProblemQP::initializeParameters] Failed to initialize linear function constraints.");
      return false;
    }

    return true;
  }
};

} /* namespace zmp */
