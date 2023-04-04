/*
 * NonLinearZmpObjectiveFunction.cpp
 *
 *  Created on: 03.26, 2017
 *      Author: Fabian Jenelten
 */

// zmp optimizer
#include <zmp_optimizer/NonLinearZmpObjectiveFunction.hpp>

// message logger
#include <message_logger/message_logger.hpp>

namespace zmp {

NonLinearZmpObjectiveFunction::NonLinearZmpObjectiveFunction(ZmpOptimizerObjectiveHandler& objectiveHandler, double hessianRegularizer)
    : QuadraticZmpObjectiveFunction(objectiveHandler, hessianRegularizer), currentObjectiveValue_(0.0), currentGradient_() {}

bool NonLinearZmpObjectiveFunction::initialize(const ZmpInfo& zmpInfo, const MotionPlan* const motionPlan) {
  return QuadraticZmpObjectiveFunction::initialize(zmpInfo, motionPlan);
}

bool NonLinearZmpObjectiveFunction::doComputationBeforeIteration(const numopt_common::Parameterization& params) {
  // Objective of QP: 0.5*x'*Q*x + f'*x
  currentObjectiveValue_ = params.getParams().dot(hessian_ * params.getParams() * 0.5 + linearTerm_);

  // Gradient of QP: Q*x + f (we assume HessianQP_ to be symmetric)
  currentGradient_ = hessian_ * params.getParams() + linearTerm_;
  return true;
}

bool NonLinearZmpObjectiveFunction::computeValue(numopt_common::Scalar& value, const numopt_common::Parameterization& /*params*/,
                                                 bool /*newParams*/) {
  value = currentObjectiveValue_;
  return true;
}

bool NonLinearZmpObjectiveFunction::getLocalGradient(numopt_common::Vector& gradient, const numopt_common::Parameterization& /*params*/,
                                                     bool /*newParams*/) {
  gradient = currentGradient_;
  return true;
}

bool NonLinearZmpObjectiveFunction::getLocalHessian(numopt_common::SparseMatrix& hessian, const numopt_common::Parameterization& /*params*/,
                                                    bool /*newParams*/) {
  hessian = hessian_.sparseView();
  return true;
}

}  // namespace zmp
