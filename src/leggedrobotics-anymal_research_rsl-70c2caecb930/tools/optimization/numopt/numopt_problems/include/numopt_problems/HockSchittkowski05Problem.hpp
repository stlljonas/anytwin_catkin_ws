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
* @file    HockSchittkowski05Problem.hpp
* @author  Anna-Maria Georgarakis, Christian Gehring
* @date    Oct. 11 2015
*/

#pragma once

#include "numopt_common/ConstrainedNonlinearProblem.hpp"

#include "numopt_common/NonlinearObjectiveFunction.hpp"
#include "numopt_common/NonlinearFunctionConstraints.hpp"


namespace numopt_problems {


/*!
 * Hock Schittkowski Collection, Problem 05
 * f(x,y) = sin(x+y) + (x-y)Â² - 1.5*x + 2.5*y + 1
 * s.t. -1.5 <= x <= 4
 *        -3 <= y <= 3
 * 2D convex non-linear function with bound constraints.
 *
 * Start: x0 = [0, 0] (feasible)
 *     f(x0) = 1
 *
 * Solution:
 *   x = -M_PI/3.0 + 0.5
 *   y = -M_PI/3.0 - 0.5
 *   f(x, y) = -0.5*sqrt(3.0)-M_PI/3.0
 *
 * Reference:
 *  W. Hock and K. Schittkowski, “Test Examples for Nonlinear Programming
 *  Codes,” Lecture Notes in Economics and Mathematical Systems, vol. 187,
 *  1981,2009.
 */
class HS05ObjectiveFunction : public numopt_common::NonlinearObjectiveFunction  {

 public:
  HS05ObjectiveFunction():NonlinearObjectiveFunction(){
  }

  virtual ~HS05ObjectiveFunction() {}

  virtual bool computeValue(numopt_common::Scalar& value, const numopt_common::Parameterization& params, bool newParams = true) {
    //return sin(x+y) + (x-y)Â² - 1.5*x + 2.5*y + 1;
    auto& p = params.getParams();
    const double x = p(0);
    const double y = p(1);
    value = sin(x+y)+(x-y)*(x-y)-1.5*x+2.5*y+1.0;
    return true;
  }
  virtual bool getLocalGradient(numopt_common::Vector& gradient, const numopt_common::Parameterization& params, bool newParams = true) {
    // J = [cos(x+y) + 2*x - 2*y - 1.5 ,  cos(x+y) - 2*x + 2*y + 2.5]
    auto& p = params.getParams();
    const double x = p(0);
    const double y = p(1);
    gradient.resize(2);
    gradient(0) = cos(x+y)+2.0*x-2.0*y-1.5;
    gradient(1) = cos(x+y)-2.0*x+2.0*y+2.5;
    return true;
  }

  virtual bool getLocalHessian(numopt_common::SparseMatrix& hessian, const numopt_common::Parameterization& params, bool newParams = true) {
    //  H =[-sin(x+y)+2 ,  -sin(x+y)-2]
    //     [-sin(x+y)-2 ,  -sin(x+y)+2]
    auto& p = params.getParams();
    const double x = p(0);
    const double y = p(1);
    hessian.resize(2,2);
    hessian.coeffRef(0,0) = -sin(x+y)+2.0;
    hessian.coeffRef(0,1) = -sin(x+y)-2.0;
    hessian.coeffRef(1,0) = -sin(x+y)-2.0;
    hessian.coeffRef(1,1) = -sin(x+y)+2.0;
    return true;
  }

};


/*!
 * -1.5 <= x <= 4
 *   -3 <= y <= 3
 */
class HS05FunctionConstraints: public numopt_common::NonlinearFunctionConstraints {
public:
  HS05FunctionConstraints():NonlinearFunctionConstraints() {
    nEqualityConstraints_ = 0;
    nInequalityConstraints_ = 0;
  }

  virtual ~HS05FunctionConstraints() {

  }

  virtual bool getGlobalBoundConstraintMinValues(numopt_common::Vector& values) {
    values.resize(2);
    values(0) = -1.5;
    values(1) = -3.0;
    return true;
  }

  virtual bool getGlobalBoundConstraintMaxValues(numopt_common::Vector& values) {
    values.resize(2);
    values(0) = 4.0;
    values(1) = 3.0;
    return true;
  }

};

class HockSchittkowski05Problem : public numopt_common::ConstrainedNonlinearProblem {
 public:
  HockSchittkowski05Problem():ConstrainedNonlinearProblem(std::shared_ptr<numopt_common::NonlinearObjectiveFunction>(new HS05ObjectiveFunction),
                                                          std::shared_ptr<numopt_common::NonlinearFunctionConstraints>(new HS05FunctionConstraints())) {

    }

  virtual ~HockSchittkowski05Problem() {}
};

} // namespace
