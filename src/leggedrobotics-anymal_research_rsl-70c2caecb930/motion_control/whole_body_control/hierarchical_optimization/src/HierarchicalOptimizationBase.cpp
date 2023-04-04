/*
 * HierarchicalOptimizationBase.cpp
 *
 *  Created on: Oct 27, 2015
 *      Author: Dario Bellicoso
 */

#include <hierarchical_optimization/HierarchicalOptimizationBase.hpp>

namespace hopt {

HierarchicalOptimizationBase::HierarchicalOptimizationBase() :
    solutionDimension_(0)
{
}

HierarchicalOptimizationBase::HierarchicalOptimizationBase(unsigned int solutionDimension) :
    solutionDimension_(solutionDimension)
{

}

bool HierarchicalOptimizationBase::resetOptimization() {
  prioritizedTasks_.clear();
  stackedTasks_.clear();
  priorityVec_.clear();
  return true;
}

bool HierarchicalOptimizationBase::appendMatrix(Eigen::MatrixXd& A, const Eigen::MatrixXd& Anew) {
  A.conservativeResize(A.rows() + Anew.rows(), Anew.cols());
  A.bottomLeftCorner(Anew.rows(), Anew.cols()) = Anew;
  return true;
}

bool HierarchicalOptimizationBase::appendRow(Eigen::MatrixXd& A, const Eigen::VectorXd& a) {
  A.conservativeResize(A.rows() + 1, A.cols());
  A.bottomRows(1) = a.transpose();
  return true;
}

bool HierarchicalOptimizationBase::buildPrioritizedOptimizationProblem() {
  // get unique ordered priorities
  std::sort(priorityVec_.begin(), priorityVec_.end());
  priorityVec_.erase(std::unique(priorityVec_.begin(), priorityVec_.end()), priorityVec_.end());

  for (const auto& prio : priorityVec_) {
    stackTasksWithPriority(prio);
  }

  return true;
}

void HierarchicalOptimizationBase::stackTasksWithPriority(int priority) {
  // Search for all task which have the specified priority and stack them.
  unsigned int eqCount = 0;
  unsigned int ineqCount = 0;

  std::vector<TaskList::iterator> iterators;

  for (auto it = prioritizedTasks_.begin(); it != prioritizedTasks_.end(); ++it) {
    if (it->getPriority() == priority) {
      eqCount += it->getEqualityConstraintJacobian().rows();
      ineqCount += it->getInequalityConstraintJacobian().rows();
      iterators.push_back(it);
    }
  }

  Eigen::MatrixXd matA = Eigen::MatrixXd(eqCount, solutionDimension_);
  Eigen::MatrixXd matD = Eigen::MatrixXd(ineqCount, solutionDimension_);
  Eigen::VectorXd vecb(eqCount);
  Eigen::VectorXd vecd(ineqCount);
  std::string description;

  unsigned int eqInsertedAt = 0u;
  unsigned int ineqInsertedAt = 0u;

  for (const auto& it : iterators) {
    const auto numEqConstraints = it->getEqualityConstraintJacobian().rows();
    const auto numIneqConstraints = it->getInequalityConstraintJacobian().rows();

    if (numEqConstraints) {
      matA.middleRows(eqInsertedAt, numEqConstraints) =
          it->getEqualityConstraintRelativeWeights().asDiagonal()*it->getEqualityConstraintJacobian();
      vecb.segment(eqInsertedAt, numEqConstraints) =
          it->getEqualityConstraintRelativeWeights().asDiagonal()*it->getEqualityConstraintTargetValues();
      eqInsertedAt += numEqConstraints;
    }

    if (numIneqConstraints) {
      matD.middleRows(ineqInsertedAt, numIneqConstraints) =
          it->getInequalityConstraintRelativeWeights().asDiagonal()*it->getInequalityConstraintJacobian();
      vecd.segment(ineqInsertedAt, numIneqConstraints) =
          it->getInequalityConstraintRelativeWeights().asDiagonal()*it->getInequalityConstraintMaxValues();
      ineqInsertedAt += numIneqConstraints;
    }

    description += it->getName();

    // Remove task from list.
    prioritizedTasks_.erase(it);
  }

  stackedTasks_.emplace_back(std::move(description), std::move(priority),
                             std::move(matA), std::move(vecb), std::move(matD), std::move(vecd),
                             ConstraintType::Mixed);
}

void HierarchicalOptimizationBase::setSolutionDimension(unsigned int solutionDimension) {
  solutionDimension_ = solutionDimension;
}

const HierarchicalOptimizationBase::TaskList& HierarchicalOptimizationBase::getOptimizationProblems() const {
  return prioritizedTasks_;
}

const HierarchicalOptimizationBase::TaskList& HierarchicalOptimizationBase::getStackedProblems() const {
  return stackedTasks_;
}

} /* namespace hopt */