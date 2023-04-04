/*
 * NonLinearZmpObjectiveFunction.hpp
 *
 *  Created on: 09.05, 2017
 *      Author: Fabian Jenelten
 */

#pragma once

// zmp optimizer
#include "zmp_optimizer/QuadraticZmpObjectiveFunction.hpp"

namespace zmp {

class NonLinearZmpObjectiveFunction : public QuadraticZmpObjectiveFunction {
  /*
   * Implementation of the objective function
   *  min_{x} f(x)
   */

 public:
  explicit NonLinearZmpObjectiveFunction(ZmpOptimizerObjectiveHandler& objectiveHandler, double hessianRegularizer);

  ~NonLinearZmpObjectiveFunction() override = default;

  bool initialize(const ZmpInfo& zmpInfo, const MotionPlan* const motionPlan) override;

  //! Computation before iteration.
  bool doComputationBeforeIteration(const numopt_common::Parameterization& params);

  // Evaluate f(x) at x*
  virtual bool computeValue(numopt_common::Scalar& value, const numopt_common::Parameterization& params, bool newParams = true) override;

  // Evaluate gradient df(x)/dx at x*
  virtual bool getLocalGradient(numopt_common::Vector& gradient, const numopt_common::Parameterization& params,
                                bool newParams = true) override;

  // Evaluate Hessian ddf(x)/ddx at x*
  virtual bool getLocalHessian(numopt_common::SparseMatrix& hessian, const numopt_common::Parameterization& params,
                               bool newParams = true) override;

 protected:
  //! Objective value at current iteration.
  double currentObjectiveValue_;

  //! Gradient at current iteration.
  numopt_common::Vector currentGradient_;
};

}  // namespace zmp
