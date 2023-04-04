/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Christian Gehring, Stelian Coros
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Autonomous Systems Lab nor ETH Zurich nor
 *     Carnegie Mellon University nor the names of its contributors
 *     may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/*!
* @file    ConstrainedNonlinearProblem.cpp
* @author  Stelian Coros, Christian Gehring
* @date    Aug 16, 2015
*/
#include "numopt_common/ConstrainedNonlinearProblem.hpp"

namespace numopt_common {



ConstrainedNonlinearProblem::ConstrainedNonlinearProblem(std::shared_ptr<NonlinearObjectiveFunction> objectiveFunction,
                                                         std::shared_ptr<NonlinearFunctionConstraints> functionConstraints) :
    objective_(objectiveFunction),
    constraints_(functionConstraints)
{
}

NonlinearObjectiveFunction* ConstrainedNonlinearProblem::getObjectiveFunctionPtr()
{
  return objective_.get();
}

NonlinearFunctionConstraints* ConstrainedNonlinearProblem::getFunctionConstraintsPtr()
{
  return constraints_.get();
}

bool ConstrainedNonlinearProblem::getLocalHessianOfLagrangian(SparseMatrix& hessian, const Parameterization& params) const {
  // Hessian of objective
  SparseMatrix localHessianObjective;
  if(!objective_->getLocalHessian(localHessianObjective, params)) {
    return false;
  }
  hessian = localHessianObjective;

  // Hessian of equality constraints
  for(int i=0;i<constraints_->getNumberOfEqualityConstraints();i++){
   SparseMatrix localEqualityConstraintHessian;
   if(!constraints_->getLocalEqualityConstraintHessian(localEqualityConstraintHessian, params, i)) {
     return false;
   }
   hessian += localEqualityConstraintHessian;
  }

  // Hessian of inequality constraints
  for(int i=0;i<constraints_->getNumberOfInequalityConstraints();i++){
   SparseMatrix localInequalityConstraintHessian;
   if(!constraints_->getLocalInequalityConstraintHessian(localInequalityConstraintHessian, params, i)) {
     return false;
   }
   hessian += localInequalityConstraintHessian;
  }

  return true;
}

bool ConstrainedNonlinearProblem::getGlobalHessianOfLagrangian(SparseMatrix& hessian, const Parameterization& params) const {
  SparseMatrix localHessian;
  if(!getLocalHessianOfLagrangian(localHessian, params)) {
    return false;
  }
  numopt_common::SparseMatrix transformMatrix;
  if (!params.getTransformMatrixGlobalToLocal(transformMatrix, params.getParams())) {
    return false;
  }
  hessian = transformMatrix.transpose()*localHessian*transformMatrix;
  return true;
}

bool ConstrainedNonlinearProblem::getLowerLocalHessianOfLagrangian(
    SparseMatrix& hessian, const Parameterization& params,
    Scalar obj_factor, const Vector& lambda) const {
  // Hessian of objective
  SparseMatrix localHessianObjective;
  if(!objective_->getLocalHessian(localHessianObjective, params)) {
    return false;
  }
  hessian = obj_factor*localHessianObjective.triangularView<Eigen::Lower>();

  // Hessian of equality constraints
  for(int i=0;i<constraints_->getNumberOfEqualityConstraints();i++){
   SparseMatrix localEqualityConstraintHessian;
   if(!constraints_->getLocalEqualityConstraintHessian(localEqualityConstraintHessian, params, i)) {
     return false;
   }
   hessian += lambda(i)*localEqualityConstraintHessian.triangularView<Eigen::Lower>();
  }

  // Hessian of inequality constraints
  for(int i=0;i<constraints_->getNumberOfInequalityConstraints();i++){
   SparseMatrix localInequalityConstraintHessian;
   if(!constraints_->getLocalInequalityConstraintHessian(localInequalityConstraintHessian, params, i)) {
     return false;
   }
   hessian += lambda(i+constraints_->getNumberOfEqualityConstraints())*
      localInequalityConstraintHessian.triangularView<Eigen::Lower>();
  }
  return true;
}

bool ConstrainedNonlinearProblem::getLowerLocalHessianOfLagrangian(SparseMatrix& hessian, const Parameterization& params) const {
  return getLowerLocalHessianOfLagrangian(hessian, params, 1.0,
      Vector::Ones(constraints_->getNumberOfEqualityConstraints() +
                   constraints_->getNumberOfInequalityConstraints()));
}

bool ConstrainedNonlinearProblem::getLowerGlobalHessianOfLagrangian(
    SparseMatrix& hessian, const Parameterization& params,
    Scalar obj_factor, const Vector& lambda) const {
  SparseMatrix localHessian;
  if(!getLowerLocalHessianOfLagrangian(localHessian, params, obj_factor, lambda)) {
    return false;
  }
  numopt_common::SparseMatrix transformMatrix;
  if (!params.getTransformMatrixGlobalToLocal(transformMatrix, params.getParams())) {
    return false;
  }
  hessian = transformMatrix.transpose()*localHessian*transformMatrix;
  return true;
}

bool ConstrainedNonlinearProblem::getLowerGlobalHessianOfLagrangian(SparseMatrix& hessian, const Parameterization& params) const {
  return getLowerGlobalHessianOfLagrangian(hessian, params, 1.0,
    Vector::Ones(constraints_->getNumberOfEqualityConstraints() +
                 constraints_->getNumberOfInequalityConstraints()));
}

} /* namespace numopt */
