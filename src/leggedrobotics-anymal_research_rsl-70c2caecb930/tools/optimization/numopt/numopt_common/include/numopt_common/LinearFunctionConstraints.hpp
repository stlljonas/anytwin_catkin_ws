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
* @file    LinearFunctionConstraints.hpp
* @author  Christian Gehring, Dario Bellicoso
* @date    Aug 16, 2015
*/
#pragma once

#include <numopt_common/NonlinearFunctionConstraints.hpp>

// Printing
#include <iostream>
#include <fstream>
#include <iomanip>

namespace numopt_common {

/*!
  Linear equality and inequality constraints
  that might be applied to an objective function.

  Equality constraints:    A*p = b
  Inequality constraints:  d <= C*p <= f
  Bound constraints:       l <= p <= u

*/
class LinearFunctionConstraints: public NonlinearFunctionConstraints
{
 public:
  LinearFunctionConstraints();
  virtual ~LinearFunctionConstraints();

  virtual bool getEqualityConstraintValues(Vector& values, const Parameterization& p, bool newParams = true);

  virtual bool getGlobalEqualityConstraintJacobian(SparseMatrix& jacobian) {
    jacobian = globalEqConJacobian_;
    return true;
  }

  const SparseMatrix& getGlobalEqualityConstraintJacobian() const { return globalEqConJacobian_; }
  virtual bool getGlobalEqualityConstraintJacobian(SparseMatrix& jacobian, const Parameterization& p, bool newParams = true);
  virtual void setGlobalEqualityConstraintJacobian(const SparseMatrix& jacobian);
  virtual bool getGlobalEqualityConstraintHessian(SparseMatrix& hessian, const Parameterization& p, int iConstraint, bool newParams = true);

  const Vector& getEqualityConstraintTargetValues() const { return eqConTargetValues_; }
  virtual bool getEqualityConstraintTargetValues(Vector& values);
  virtual void setEqualityConstraintTargetValues(const Vector& values);

  virtual bool getInequalityConstraintValues(Vector& values, const Parameterization& p, bool newParams = true);

  const Vector& getInequalityConstraintMinValues() const { return ineqConMinValues_; }
  virtual bool getInequalityConstraintMinValues(Vector& values) {
    values = ineqConMinValues_;
    return true;
  }
  virtual void setInequalityConstraintMinValues(const Vector& d);

  const Vector& getInequalityConstraintMaxValues() const { return ineqConMaxValues_; }
  virtual bool getInequalityConstraintMaxValues(Vector& values) {
    values = ineqConMaxValues_;
    return true;
  }
  virtual void setInequalityConstraintMaxValues(const Vector& f);

  const SparseMatrix& getGlobalInequalityConstraintJacobian() const { return globalIneqConJacobian_; };
  virtual bool getGlobalInequalityConstraintJacobian(numopt_common::SparseMatrix& jacobian, const Parameterization& p, bool newParams = true);
  virtual void setGlobalInequalityConstraintJacobian(const numopt_common::SparseMatrix& jacobian);
  virtual bool getGlobalInequalityConstraintHessian(numopt_common::SparseMatrix& hessian, const Parameterization& p, int iConstraint, bool newParams = true);

  const Vector& getGlobalBoundConstraintMinValues() const { return globalMinBounds_; }
  virtual bool getGlobalBoundConstraintMinValues(Vector& values) {
    values = globalMinBounds_;
    return true;
  }
  virtual void setGlobalBoundConstraintMinValues(const Vector& l);

  const Vector& getGlobalBoundConstraintMaxValues() const { return globalMaxBounds_; }
  virtual bool getGlobalBoundConstraintMaxValues(Vector& values) {
    values = globalMaxBounds_;
    return true;
  }
  virtual void setGlobalBoundConstraintMaxValues(const Vector& u);

  virtual void append(const LinearFunctionConstraints & linFunCon);

  virtual void transformMinToMaxInequalityConstraints();

  bool printToFile(std::ofstream& file) const;

 protected:
  Vector eqConTargetValues_;
  SparseMatrix globalEqConJacobian_;

  Vector ineqConMinValues_;
  Vector ineqConMaxValues_;
  SparseMatrix globalIneqConJacobian_;


  Vector globalMinBounds_;
  Vector globalMaxBounds_;
};

} /* namespace numopt_common */
