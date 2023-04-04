/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Christian Gehring, Stelian Coros
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
* @file    HockSchittkowski03Problem.hpp
* @author  Christian Gehring
* @date    Feb. 2016
*/

#pragma once

#include "numopt_common/QuadraticProblem.hpp"

#include "numopt_common/QuadraticObjectiveFunction.hpp"
#include "numopt_common/WeightedSumQuadraticObjectiveFunction.hpp"
#include "numopt_common/L2Regularizer.hpp"
#include "numopt_common/LinearFunctionConstraints.hpp"


namespace numopt_problems {


/*!
 * Hock Schittkowski Collection, Problem 03
 * f(x) = x2 + 1e-5*(x2 - x1)^2
 * s.t. 0 <= x2
 *
 * 2D convex quadratic function with bound constraints.
 *
 * Start: x0 = [10, 1] (feasible)
 *     f(x0) = 1.00081
 *
 * Solution:
 *        x* = [0, 0]
 *     f(x*) = 0
 *
 * Reference:
 *  W. Hock and K. Schittkowski, “Test Examples for Nonlinear Programming
 *  Codes,” Lecture Notes in Economics and Mathematical Systems, vol. 187,
 *  1981,2009.
 */
class HS03ObjectiveFunction : public numopt_common::QuadraticObjectiveFunction  {
 public:
  HS03ObjectiveFunction():QuadraticObjectiveFunction(){
    numopt_common::SparseMatrix Q;
    Q.resize(2,2);
    Q.coeffRef(0,0) = 2.0*1.0e-5;
    Q.coeffRef(0,1) = -2.0*1.0e-5;
    Q.coeffRef(1,0) = -2.0*1.0e-5;
    Q.coeffRef(1,1) = 2.0*1.0e-5;
    setGlobalHessian(Q);

    numopt_common::Vector c;
    c.resize(2);
    c(0) = 0.0;
    c(1) = 1.0;
    setLinearTerm(c);

  }

  virtual ~HS03ObjectiveFunction() {}

};

/*!
 * -1.5 <= x <= 4
 *   -3 <= y <= 3
 */
class HS03FunctionConstraints: public numopt_common::LinearFunctionConstraints {
public:
  HS03FunctionConstraints():LinearFunctionConstraints() {
    // bounds
    globalMinBounds_.resize(2);
    globalMaxBounds_.resize(2);
    globalMinBounds_(0) = -std::numeric_limits<double>::max();
    globalMinBounds_(1) = 0.0;
    globalMaxBounds_(0) = std::numeric_limits<double>::max();
    globalMaxBounds_(1) = std::numeric_limits<double>::max();

    // equality constraints
    nEqualityConstraints_ = 0;
    globalEqConJacobian_.resize(nEqualityConstraints_,2);

    // inequality constraints
    nInequalityConstraints_ = 0;
    globalIneqConJacobian_.resize(nInequalityConstraints_,2);
    ineqConMaxValues_.resize(0);
    ineqConMinValues_.resize(0);
  }
  virtual ~HS03FunctionConstraints() {

  }

};



class HockSchittkowski03Problem : public numopt_common::QuadraticProblem {
 public:
  HockSchittkowski03Problem(double regularizer = 0.0):QuadraticProblem(std::shared_ptr<numopt_common::QuadraticObjectiveFunction>(
                                               new numopt_common::WeightedSumQuadraticObjectiveFunction(
                                                   std::vector<std::shared_ptr<numopt_common::QuadraticObjectiveFunction>>{
                                                              std::shared_ptr<numopt_common::QuadraticObjectiveFunction>(new HS03ObjectiveFunction()),
                                                              std::shared_ptr<numopt_common::QuadraticObjectiveFunction>(new numopt_common::L2Regularizer(2, regularizer))
                                                    })),
                                               std::shared_ptr<numopt_common::LinearFunctionConstraints>(new HS03FunctionConstraints())) {

    }

  virtual ~HockSchittkowski03Problem() {}
};



} // namespace
