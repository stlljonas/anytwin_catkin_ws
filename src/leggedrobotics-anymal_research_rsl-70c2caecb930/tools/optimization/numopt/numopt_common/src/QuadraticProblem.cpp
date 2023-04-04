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
* @file    QuadraticProblem.cpp
* @author  Christian Gehring
* @date    Aug 16, 2015
*/

// numopt_common
#include "numopt_common/QuadraticProblem.hpp"

namespace numopt_common {

QuadraticProblem::QuadraticProblem(std::shared_ptr<QuadraticObjectiveFunction> objectiveFunction,
                                   std::shared_ptr<LinearFunctionConstraints> functionConstraints):
    ConstrainedNonlinearProblem(objectiveFunction, functionConstraints)
{

}

QuadraticProblem::~QuadraticProblem()
{
}

QuadraticObjectiveFunction* QuadraticProblem::getQuadraticObjectiveFunctionPtr() {
  return static_cast<QuadraticObjectiveFunction*>(objective_.get());
}

LinearFunctionConstraints* QuadraticProblem::getLinearFunctionConstraintsPtr() {
  return static_cast<LinearFunctionConstraints*>(constraints_.get());
}

void QuadraticProblem::setOptimizationMatrices(
    const numopt_common::SparseMatrix& globalHessian,
    const numopt_common::Vector& linearTerm,
    const numopt_common::SparseMatrix& globalEqConJacobian,
    const numopt_common::SparseMatrix& globalIneqConJacobian,
    const numopt_common::Vector& eqConTargetValues,
    const numopt_common::Vector& ineqConMinValues,
    const numopt_common::Vector& ineqConMaxValues,
    const numopt_common::Vector& globalMinBounds,
    const numopt_common::Vector& globalMaxBounds) {

  getQuadraticObjectiveFunctionPtr()->setGlobalHessian(globalHessian);
  getQuadraticObjectiveFunctionPtr()->setLinearTerm(linearTerm);

  getLinearFunctionConstraintsPtr()->setGlobalEqualityConstraintJacobian(globalEqConJacobian);
  getLinearFunctionConstraintsPtr()->setGlobalInequalityConstraintJacobian(globalIneqConJacobian);

  getLinearFunctionConstraintsPtr()->setEqualityConstraintTargetValues(eqConTargetValues);
  getLinearFunctionConstraintsPtr()->setInequalityConstraintMinValues(ineqConMinValues);


  getLinearFunctionConstraintsPtr()->setInequalityConstraintMaxValues(ineqConMaxValues);
  getLinearFunctionConstraintsPtr()->setGlobalBoundConstraintMinValues(globalMinBounds);
  getLinearFunctionConstraintsPtr()->setGlobalBoundConstraintMaxValues(globalMaxBounds);

  getLinearFunctionConstraintsPtr()->setNumberOfEqualityConstraints(eqConTargetValues.size());
  getLinearFunctionConstraintsPtr()->setNumberOfInequalityConstraints(ineqConMinValues.size());
}

void QuadraticProblem::setOptimizationMatrices(
    const numopt_common::SparseMatrix& globalHessian,
    const numopt_common::Vector& linearTerm,
    const numopt_common::SparseMatrix& globalEqConJacobian,
    const numopt_common::SparseMatrix& globalIneqConJacobian,
    const numopt_common::Vector& eqConTargetValues,
    const numopt_common::Vector& ineqConMinValues,
    const numopt_common::Vector& globalMinBounds,
    const numopt_common::Vector& globalMaxBounds) {

  Eigen::VectorXd ineqConMaxValues;
  ineqConMaxValues.setConstant(ineqConMinValues.size(), std::numeric_limits<double>::max());

  setOptimizationMatrices(
      globalHessian,
      linearTerm,
      globalEqConJacobian,
      globalIneqConJacobian,
      eqConTargetValues,
      ineqConMinValues,
      ineqConMaxValues,
      globalMinBounds,
      globalMaxBounds
  );
}

void QuadraticProblem::setOptimizationMatrices(
    const numopt_common::SparseMatrix& globalHessian,
    const numopt_common::Vector& linearTerm,
    const numopt_common::SparseMatrix& globalEqConJacobian,
    const numopt_common::SparseMatrix& globalIneqConJacobian,
    const numopt_common::Vector& eqConTargetValues,
    const numopt_common::Vector& ineqConMinValues) {

  Eigen::VectorXd globalMaxBounds;
  globalMaxBounds.setConstant(linearTerm.size(), std::numeric_limits<double>::max());

  setOptimizationMatrices(
      globalHessian,
      linearTerm,
      globalEqConJacobian,
      globalIneqConJacobian,
      eqConTargetValues,
      ineqConMinValues,
      -globalMaxBounds,
      globalMaxBounds
  );
}

bool QuadraticProblem::printToFile(const std::string& filename) {
  std::ofstream file(filename);
  if (!file) { return false; }
  if(!getQuadraticObjectiveFunctionPtr()->printToFile(file)) { return false; }
  if(!getLinearFunctionConstraintsPtr()->printToFile(file))  { return false; }
  file.close();
  return true;
}


} /* namespace numopt_common */
