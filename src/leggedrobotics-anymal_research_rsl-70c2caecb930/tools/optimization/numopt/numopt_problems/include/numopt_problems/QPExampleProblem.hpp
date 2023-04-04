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
* @file    QuadObjAndLinCon.hpp
* @author  Stelian Coros, Christian Gehring
* @date    Aug 16, 2015
*/
#pragma once

#include "numopt_common/numopt_common.hpp"
#include "numopt_common/QuadraticProblem.hpp"

namespace numopt_problems {

/*!
 *  f(x,y) = -8*x - 16*y + x^2 + 4*y^2;
 */
class QPExampleObjectiveFunction : public numopt_common::QuadraticObjectiveFunction {
 public:
  QPExampleObjectiveFunction():QuadraticObjectiveFunction() {
    numopt_common::SparseMatrix Q;

    Q.resize(2,2);
    /*  H =
      [ 2, 0]
      [ 0, 8]
    */
    Q.coeffRef(0,0) = 2;
    Q.coeffRef(1,1) = 8;

    setGlobalHessian(Q);
    numopt_common::Vector c;
    c.resize(2);
    c(0) = -8.0;
    c(1) = -16.0;
    setLinearTerm(c);
  }
  virtual ~QPExampleObjectiveFunction() {}
//  virtual double computeValue(const Eigen::VectorXd& p) {
//    return -8*p(0) - 16*p(1) + p(0)*p(0) + 4*p(1)*p(1);
//  }
//  virtual const numopt_common::SparseMatrix& getHessianAt(const Eigen::VectorXd& p) {
//    // constant
//    return hessian_;
//  }
//  virtual const Eigen::VectorXd& getGradientAt(const Eigen::VectorXd& p) {
//    /* J = [ 2*x - 8, 8*y - 16] */
//    gradient_(0) = 2*p(0) - 8;
//    gradient_(1) = 8*p(1) - 16;
//    return gradient_;
//  }

};

/*!
 * x+y <= 5
 * x <= 3
 * x >= 0
 * y >= 0
 *
 * C = [1 1
 *      1 0]
 *
 * f = [5, 3]'
 */
class QPExampleFunctionConstraints: public numopt_common::LinearFunctionConstraints {
public:
  QPExampleFunctionConstraints():LinearFunctionConstraints() {

    const int nParams = 2;
    // equality constraints
    nEqualityConstraints_ = 0;
    globalEqConJacobian_.resize(nEqualityConstraints_,nParams);

    // inequality constraints
    nInequalityConstraints_ = 2;
    globalIneqConJacobian_.resize(nInequalityConstraints_,nParams);
    globalIneqConJacobian_.coeffRef(0,0) = 1.0;
    globalIneqConJacobian_.coeffRef(0,1) = 1.0;
    globalIneqConJacobian_.coeffRef(1,0) = 1.0;

    ineqConMinValues_ = -std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(nInequalityConstraints_);
    ineqConMaxValues_.resize(nInequalityConstraints_);
    ineqConMaxValues_(0) = 5;
    ineqConMaxValues_(1) = 3;

    // bounds
    globalMinBounds_.setZero(nParams);
    globalMaxBounds_ = std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(nParams);

  }

  virtual ~QPExampleFunctionConstraints() {}

};

class QPExampleProblem : public numopt_common::QuadraticProblem {
 public:
  QPExampleProblem():QuadraticProblem(std::shared_ptr<numopt_common::QuadraticObjectiveFunction>(new QPExampleObjectiveFunction()),
                                      std::shared_ptr<numopt_common::LinearFunctionConstraints>(new QPExampleFunctionConstraints())) {
  }
  virtual ~QPExampleProblem() {

  }

};

} // namespace numopt_problems
