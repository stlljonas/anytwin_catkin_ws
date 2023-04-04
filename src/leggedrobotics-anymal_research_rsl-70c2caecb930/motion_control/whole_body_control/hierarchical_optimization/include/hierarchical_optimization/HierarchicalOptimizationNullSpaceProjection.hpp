/*
 * HierarchicalOptimization.hpp
 *
 *  Created on: Sep 4, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

#include <hierarchical_optimization/HierarchicalOptimizationBase.hpp>

namespace hopt {

class HierarchicalOptimizationNullSpaceProjection final : public HierarchicalOptimizationBase {
 private:
  using Base = HierarchicalOptimizationBase;

 public:
  HierarchicalOptimizationNullSpaceProjection() = default;
  ~HierarchicalOptimizationNullSpaceProjection() override = default;

  explicit HierarchicalOptimizationNullSpaceProjection(unsigned int solutionDimension);

  bool solveOptimization(Eigen::VectorXd& solution) override;
};

} /* namespace hopt */
