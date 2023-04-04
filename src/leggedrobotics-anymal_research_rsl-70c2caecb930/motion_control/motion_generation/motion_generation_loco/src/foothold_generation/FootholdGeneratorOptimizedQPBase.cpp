/*
 * FootholdGeneratorOptimizedQPBase.cpp
 *
 *  Created on: Mar 15, 2018
 *      Author: Fabian Jenelten
 */

// loco
#include "loco/foothold_generation/FootholdGeneratorOptimizedQPBase.hpp"

namespace loco {

FootholdGeneratorOptimizedQPBase::FootholdGeneratorOptimizedQPBase()
    : Base(),
      costFunctionHessian_(),
      costFunctionLinearTerm_(),
      equalityConstraintJacobian_(),
      equalityConstraintTargetValues_(),
      inequalityConstraintJacobian_(),
      inequalityConstraintMinValues_(),
      solutionFootholdInPlaneFrame_(),
      optimizationProblemQP_(nullptr),
      qpSolver_(nullptr),
      solutionDimension_(2u),
      numOfEqualityConstraints_(0u),
      numOfInequalityConstraints_(0u),
      equalityConstraintsCounter_(0u),
      inequalityConstraintsCounter_(0u)
{

}

bool FootholdGeneratorOptimizedQPBase::loadParameters(const TiXmlHandle& handle) {
  return true;
}

bool FootholdGeneratorOptimizedQPBase::initialize(double dt) {
  // Initialize zmp optimization problem (linear optimization)
  optimizationProblemQP_.reset(new numopt_common::QuadraticProblem(
      std::shared_ptr<numopt_common::QuadraticObjectiveFunction>(new numopt_common::QuadraticObjectiveFunction),
      std::shared_ptr<numopt_common::LinearFunctionConstraints>(new numopt_common::LinearFunctionConstraints)
    )
  );

  qpSolver_.reset(new numopt_quadprog::ActiveSetFunctionMinimizer(100));

  // By default: no equality constraints.
  numOfEqualityConstraints_ = 0u;
  equalityConstraintsCounter_ = 0u;
  equalityConstraintJacobian_.resize(numOfEqualityConstraints_, solutionDimension_);
  equalityConstraintTargetValues_.resize(numOfEqualityConstraints_);

  return true;
}

void FootholdGeneratorOptimizedQPBase::addObjectiveForLeg(
    const Eigen::Matrix<double,2,2>& equalityJacobian,
    const Eigen::Vector2d& equalityTargetValues,
    const Weight& taskWeights) {
  costFunctionHessian_    += equalityJacobian.transpose()*taskWeights*equalityJacobian;
  costFunctionLinearTerm_ -= equalityJacobian.transpose()*taskWeights*equalityTargetValues;
}


void FootholdGeneratorOptimizedQPBase::addApproachingObjective(
    const Eigen::Vector2d& equalityTargetValues,
    const Weight& taskWeights) {
  costFunctionHessian_    += taskWeights;
  costFunctionLinearTerm_ -= taskWeights*equalityTargetValues;
}

void FootholdGeneratorOptimizedQPBase::stop() {
  if(qpSolver_ != nullptr) {qpSolver_->stop(); }
}


} /* namespace loco */
