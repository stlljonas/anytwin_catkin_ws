/*
 * HierarchicalOptimizationQpCascade.hpp
 *
 *  Created on: Oct 27, 2015
 *      Author: dbellicoso
 */

#pragma once

#include <hierarchical_optimization/HierarchicalOptimizationBase.hpp>
#include <numopt_common/QuadraticProblemSolver.hpp>

#include <std_utils/std_utils.hpp>

#include <parameter_handler/parameter_handler.hpp>

namespace hopt {

class HierarchicalOptimizationQpCascade final : public HierarchicalOptimizationBase
{
 public:
  explicit HierarchicalOptimizationQpCascade(int solutionDimension,
                                             std::unique_ptr<numopt_common::QuadraticProblemSolver>&& minimizer);
  ~HierarchicalOptimizationQpCascade() override = default;

  bool solveOptimization(Eigen::VectorXd& solution) override;

 protected:
  // numerical solver
  std::unique_ptr<numopt_common::QuadraticProblemSolver> minimizer_;
  std::shared_ptr<numopt_common::QuadraticObjectiveFunction> costFunction_;
  std::shared_ptr<numopt_common::LinearFunctionConstraints> functionConstraints_;
  numopt_common::QuadraticProblem quadraticProblem_;

  std_utils::HighResolutionClockTimer timer_;
};

} /* namespace hopt */

