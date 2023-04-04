/*
 * HierarchicalOptimization.cpp
 *
 *  Created on: Sep 4, 2015
 *      Author: Dario Bellicoso
 */

// hierarchical optimization
#include <hierarchical_optimization/HierarchicalOptimizationNullSpaceProjection.hpp>

// eigen
#include <Eigen/LU>

#include <kindr/math/LinearAlgebra.hpp>

namespace hopt {

HierarchicalOptimizationNullSpaceProjection::HierarchicalOptimizationNullSpaceProjection(
    unsigned int solutionDimension)
    : Base(solutionDimension)
{

}

bool HierarchicalOptimizationNullSpaceProjection::solveOptimization(Eigen::VectorXd& sol) {
  buildPrioritizedOptimizationProblem();

  auto it = stackedTasks_.begin();
  Eigen::MatrixXd fullMat = it->getEqualityConstraintJacobian();

  Eigen::MatrixXd pseudoInverse;
  kindr::pseudoInverse(it->getEqualityConstraintJacobian(), pseudoInverse);
  sol = pseudoInverse*it->getEqualityConstraintTargetValues();

  Eigen::MatrixXd previousNullSpaceBasis = it->getEqualityConstraintJacobian().fullPivLu().kernel();

  for (size_t k=1; k<stackedTasks_.size(); ++k) {
    std::advance(it, 1);
    const Eigen::MatrixXd& equalityConstraintJacobian = it->getEqualityConstraintJacobian();
    const Eigen::VectorXd& equalityConstraintTargetValues = it->getEqualityConstraintTargetValues();

    kindr::pseudoInverse<Eigen::MatrixXd>(equalityConstraintJacobian*previousNullSpaceBasis, pseudoInverse);
    sol += previousNullSpaceBasis*pseudoInverse*(equalityConstraintTargetValues - equalityConstraintJacobian*sol);

    if (k<stackedTasks_.size()-1) {
      appendMatrix(fullMat, equalityConstraintJacobian);
      previousNullSpaceBasis = fullMat.fullPivLu().kernel();
    }

  }

  return true;
}


} /* namespace hopt */
