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
* @file    LinearFunctionConstraints.cpp
* @author  Stelian Coros, Christian Gehring, Dario Bellicoso
* @date    Aug 16, 2015
*/

#include "numopt_common/LinearFunctionConstraints.hpp"

namespace numopt_common {

LinearFunctionConstraints::LinearFunctionConstraints()
{

}

LinearFunctionConstraints::~LinearFunctionConstraints()
{
}

bool LinearFunctionConstraints::getGlobalEqualityConstraintHessian(numopt_common::SparseMatrix& hessian, const Parameterization& p, int iConstraint, bool newParams) {
  // overwrite finite-difference method since the hessians are always zero for linear constraints
  hessian.resize(p.getGlobalSize(), p.getGlobalSize());
  return true;
}

bool LinearFunctionConstraints::getGlobalInequalityConstraintHessian(numopt_common::SparseMatrix& hessian, const Parameterization& p, int iConstraint, bool newParams) {
  // overwrite finite-difference method since the hessians are always zero for linear constraints
  hessian.resize(p.getGlobalSize(), p.getGlobalSize());
  return true;
}

void LinearFunctionConstraints::setGlobalEqualityConstraintJacobian(
    const numopt_common::SparseMatrix& jacobian)
{
  globalEqConJacobian_ = jacobian;
}

void LinearFunctionConstraints::setGlobalInequalityConstraintJacobian(
    const numopt_common::SparseMatrix& jacobian)
{
  globalIneqConJacobian_ = jacobian;
}

bool LinearFunctionConstraints::getEqualityConstraintTargetValues(Vector& values)
{
  values = eqConTargetValues_;
  return true;
}

void LinearFunctionConstraints::setEqualityConstraintTargetValues(const Vector& values)
{
  eqConTargetValues_ = values;
}

void LinearFunctionConstraints::setInequalityConstraintMinValues(const Vector& p)
{
  ineqConMinValues_ = p;
}

void LinearFunctionConstraints::setInequalityConstraintMaxValues(const Vector& p)
{
  ineqConMaxValues_ = p;
}

void LinearFunctionConstraints::setGlobalBoundConstraintMinValues(const Vector& values)
{
  globalMinBounds_ = values;
}

void LinearFunctionConstraints::setGlobalBoundConstraintMaxValues(const numopt_common::Vector& p)
{
  globalMaxBounds_ = p;
}


bool LinearFunctionConstraints::getGlobalEqualityConstraintJacobian(numopt_common::SparseMatrix& jacobian,
    const Parameterization& p,
    bool newParams)
{
  jacobian = globalEqConJacobian_;
  return true;
}

bool LinearFunctionConstraints::getGlobalInequalityConstraintJacobian(numopt_common::SparseMatrix& jacobian,
                                                                 const Parameterization&p, bool newParams)
{
  jacobian = globalIneqConJacobian_;
  return true;
}

bool LinearFunctionConstraints::getEqualityConstraintValues(Vector& values, const Parameterization& params, bool newParams) {
  values = globalEqConJacobian_*params.getParams();
  return true;
}

bool LinearFunctionConstraints::getInequalityConstraintValues(Vector& values,
    const Parameterization& params, bool newParams)
{
  values = globalIneqConJacobian_*params.getParams();
  return true;
}

void LinearFunctionConstraints::append(const LinearFunctionConstraints & linFunCon) {
  // Append equality constraint target values
  const Vector& eqConTargetValuesToAppend = linFunCon.getEqualityConstraintTargetValues();
  const unsigned int eqConTargetValueRows = eqConTargetValues_.rows();
  eqConTargetValues_.conservativeResize(eqConTargetValueRows + eqConTargetValuesToAppend.rows());
  eqConTargetValues_.segment(eqConTargetValueRows, eqConTargetValuesToAppend.rows()) = eqConTargetValuesToAppend;

  // Append global equality constraint jacobian
  const SparseMatrix& globalEqConJacobianToAppend = linFunCon.getGlobalEqualityConstraintJacobian();
  // Size is zero or cols have to be same size
  assert( (globalEqConJacobian_.size() == 0) || (globalEqConJacobian_.cols() == globalEqConJacobianToAppend.cols()) );
  const unsigned int globalEqConJacobianRows = globalEqConJacobian_.rows();
  globalEqConJacobian_.conservativeResize(globalEqConJacobianRows + globalEqConJacobianToAppend.rows(), globalEqConJacobianToAppend.cols());
  globalEqConJacobian_.middleRows(globalEqConJacobianRows, globalEqConJacobianToAppend.rows()) = globalEqConJacobianToAppend;

  // Append inequality constraint min values
  const Vector& ineqConMinValuesToAppend = linFunCon.getInequalityConstraintMinValues();
  const unsigned int ineqConMinValueRows = ineqConMinValues_.rows();
  ineqConMinValues_.conservativeResize(ineqConMinValueRows + ineqConMinValuesToAppend.rows());
  ineqConMinValues_.segment(ineqConMinValueRows, ineqConMinValuesToAppend.rows()) = ineqConMinValuesToAppend;

  // Append inequality constraint max values
  const Vector& ineqConMaxValuesToAppend = linFunCon.getInequalityConstraintMaxValues();
  const unsigned int ineqConMaxValueRows = ineqConMaxValues_.rows();
  ineqConMaxValues_.conservativeResize(ineqConMaxValueRows + ineqConMaxValuesToAppend.rows());
  ineqConMaxValues_.segment(ineqConMaxValueRows, ineqConMaxValuesToAppend.rows()) = ineqConMaxValuesToAppend;

  // Append global inequality constraint jacobian
  const SparseMatrix& globalIneqConJacobianToAppend = linFunCon.getGlobalInequalityConstraintJacobian();
  // Size is zero or cols have to be same size
  assert( (globalIneqConJacobian_.size() == 0) || (globalIneqConJacobian_.cols() == globalIneqConJacobianToAppend.cols()) );
  const unsigned int globalIneqConJacobianRows = globalIneqConJacobian_.rows();
  globalIneqConJacobian_.conservativeResize(globalIneqConJacobianRows + globalIneqConJacobianToAppend.rows(), globalIneqConJacobianToAppend.cols());
  globalIneqConJacobian_.middleRows(globalIneqConJacobianRows, globalIneqConJacobianToAppend.rows()) = globalIneqConJacobianToAppend;

  // Append global min bound values
  const Vector& globalMinBoundsToAppend = linFunCon.getGlobalBoundConstraintMinValues();
  const unsigned int globalMinBoundsRows = globalMinBounds_.rows();
  globalMinBounds_.conservativeResize(globalMinBoundsRows + globalMinBoundsToAppend.rows());
  globalMinBounds_.segment(globalMinBoundsRows, globalMinBoundsToAppend.rows()) = globalMinBoundsToAppend;

  // Append global max bound values
  const Vector& globalMaxBoundsToAppend = linFunCon.getGlobalBoundConstraintMaxValues();
  const unsigned int globalMaxBoundsRows = globalMaxBounds_.rows();
  globalMaxBounds_.conservativeResize(globalMaxBoundsRows + globalMaxBoundsToAppend.rows());
  globalMaxBounds_.segment(globalMaxBoundsRows, globalMaxBoundsToAppend.rows()) = globalMaxBoundsToAppend;
}



void LinearFunctionConstraints::transformMinToMaxInequalityConstraints() {

  // Get indices of minimal function constraints
  const Eigen::Array<bool, Eigen::Dynamic, 1> indicesMinConstraints = ( ineqConMinValues_.array() != -std::numeric_limits<double>::max() );

  if( indicesMinConstraints.count() != 0) {
    const Eigen::Array<bool, Eigen::Dynamic, 1> indicesMaxConstraints = ( ineqConMaxValues_.array() != std::numeric_limits<double>::max() );
    const unsigned int nNonZeroConstraint = ( indicesMinConstraints || indicesMaxConstraints ).count();
    const unsigned int nConstraints = indicesMinConstraints.count() + indicesMaxConstraints.count();
    const unsigned int nInequalities = ineqConMinValues_.rows();

    globalIneqConJacobian_.conservativeResize(nConstraints, globalIneqConJacobian_.cols());
    ineqConMaxValues_.conservativeResize(nConstraints);
    ineqConMinValues_.conservativeResize(nConstraints);

    for( unsigned int i = 0, j = 0, k = 0; i < nInequalities; ++i ) {
      bool hasMaxConstraints = indicesMaxConstraints(i);
      if(hasMaxConstraints) {
        globalIneqConJacobian_.row(k) = globalIneqConJacobian_.row(i).eval();
        ineqConMaxValues_(k) = ineqConMaxValues_(i);
        ++k;
      }
      if( indicesMinConstraints(i) ) {
        if(hasMaxConstraints) {
          globalIneqConJacobian_.row(nNonZeroConstraint + j) = ( -globalIneqConJacobian_.row(i) ).eval();
          ineqConMaxValues_(nNonZeroConstraint + j) = -ineqConMinValues_(i);
          ++j;
        } else {
          globalIneqConJacobian_.row(k) = ( -globalIneqConJacobian_.row(i) ).eval();
          ineqConMaxValues_(k) = -ineqConMinValues_(i);
          ++k;
        }
      }
    }

    ineqConMinValues_.setConstant(-std::numeric_limits<double>::max());
  }
}


bool LinearFunctionConstraints::printToFile(std::ofstream& file) const {
  file << "[eq_jacobian] " << std::to_string((int)globalEqConJacobian_.rows()) << " " << std::to_string((int)globalEqConJacobian_.cols()) << "\n"
       << std::scientific << globalEqConJacobian_.toDense() << "\n";

  file << "[ineq_jacobian] " << std::to_string((int)globalIneqConJacobian_.rows()) << " " << std::to_string((int)globalIneqConJacobian_.cols()) << "\n"
       << std::scientific << globalIneqConJacobian_.toDense() << "\n";


  file << "[eq_target] " << std::to_string((int)eqConTargetValues_.size()) << " " << std::to_string(1) << "\n"
       << std::scientific << eqConTargetValues_  << "\n";

  file << "[ineq_min] " << std::to_string((int)ineqConMinValues_.size()) << " " << std::to_string(1) << "\n"
       << std::scientific << ineqConMinValues_  << "\n";

  file << "[ineq_max] " << std::to_string((int)ineqConMaxValues_.size()) << " " << std::to_string(1) << "\n"
       << std::scientific << ineqConMaxValues_  << "\n";

  file << "[min_bound] " << std::to_string((int)globalMinBounds_.size()) << " " << std::to_string(1) << "\n"
       << std::scientific << globalMinBounds_  << "\n";

  file << "[max_bound] " << std::to_string((int)globalMaxBounds_.size()) << " " << std::to_string(1) << "\n"
       << std::scientific << globalMaxBounds_  << "\n";

  return true;
}


} /* namespace numopt_common */
