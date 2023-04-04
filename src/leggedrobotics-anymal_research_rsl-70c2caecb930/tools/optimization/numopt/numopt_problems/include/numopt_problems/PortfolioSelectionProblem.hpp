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
* @file    PortfolioSelectionObj.hpp
* @author  Christian Gehring, Stelian Coros
* @date    Aug 16, 2015
*/
#pragma once

#include "numopt_common/numopt_common.hpp"
#include "numopt_common/ConstrainedNonlinearProblem.hpp"


namespace numopt_problems {


/*!
 * f(x,y) = -(20x + 16y - theta(2x^2 + y^2 + (x + y)^2))
 * with theta = 4/5
 */
class PortfolioSelectionObjectiveFunction : public numopt_common::NonlinearObjectiveFunction {
 public:
  PortfolioSelectionObjectiveFunction(double p_theta):NonlinearObjectiveFunction(),
  theta(p_theta){
  }
  virtual ~PortfolioSelectionObjectiveFunction() {}

  virtual bool computeValue(numopt_common::Scalar& value, const numopt_common::Parameterization& params, bool newParams = true) {
    //return -(20*p(0) + 16*p(1) - theta*(2*p(0)*p(0) + p(1)*p(1) + (p(0) + p(1))*(p(0) + p(1))));
    auto& p = params.getParams();
    const double x = p(0);
    const double y = p(1);
    value =  x*-2.0E1-y*1.6E1+theta*(pow(x+y,2.0)+(x*x)*2.0+y*y);
    return true;
  }

  virtual bool getLocalGradient(numopt_common::Vector& gradient, const numopt_common::Parameterization& params, bool newParams = true) {
    // J = [ theta*(6*x + 2*y) - 20, theta*(2*x + 4*y) - 16]
    auto& p = params.getParams();
    gradient.resize(2);
    gradient(0) = theta*(6*p(0) + 2*p(1)) - 20;
    gradient(1) = theta*(2*p(0) + 4*p(1)) - 16;
    return true;
  }

  virtual bool getLocalHessian(numopt_common::SparseMatrix& hessian, const numopt_common::Parameterization& params, bool newParams = true) {
    //  H =[ 6*theta, 2*theta]
    //     [ 2*theta, 4*theta]
    auto& p = params.getParams();

    hessian.resize(2,2);
    hessian.coeffRef(0,0) = 6*theta;
    hessian.coeffRef(0,1) = 2*theta;
    hessian.coeffRef(1,0) = 2*theta;
    hessian.coeffRef(1,1) = 4*theta;
    return true;
  }

 protected:
  double theta;

};


/*!
 * x+y <= 5
 * x >= 0
 * y >= 0
 *
 * C = [1 1]
 *
 * f = [5]
 */
class PortfolioSelectionFunctionConstraints: public numopt_common::NonlinearFunctionConstraints {
public:
  PortfolioSelectionFunctionConstraints():NonlinearFunctionConstraints() {
    // equality constraints
    nEqualityConstraints_ = 0;

    // inequality constraints
    nInequalityConstraints_ = 1;
  }
  virtual ~PortfolioSelectionFunctionConstraints() {

  }

  bool getLocalInequalityConstraintJacobian(numopt_common::SparseMatrix& jacobian,
                                             const numopt_common::Parameterization& params, bool newParams = true) {
    jacobian.resize(1,2);
    jacobian.coeffRef(0,0) = 1.0;
    jacobian.coeffRef(0,1) = 1.0;
    return true;
  }

  virtual bool getInequalityConstraintMinValues(numopt_common::Vector& values) {
    values = -std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(1);
    return true;
  }

  virtual bool getInequalityConstraintMaxValues(numopt_common::Vector& values) {
    values.resize(1);
    values(0) = 5.0;
    return true;
  }

  virtual bool getGlobalBoundConstraintMinValues(numopt_common::Vector& values) {
    values.setZero(2);
    return true;
  }

  virtual bool getGlobalBoundConstraintMaxValues(numopt_common::Vector& values) {
    values = std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(2);
    return true;
  }

  virtual bool getInequalityConstraintValues(numopt_common::Vector& values,
                                             const numopt_common::Parameterization& params, bool newParams = true) {
    values.resize(1);
    values(0) = params.getParams()(0) +  params.getParams()(1);
    return true;
  }

};


class PortfolioSelectionProblem : public numopt_common::ConstrainedNonlinearProblem {
 public:
  PortfolioSelectionProblem(double theta):
    ConstrainedNonlinearProblem(std::shared_ptr<numopt_common::NonlinearObjectiveFunction>(new PortfolioSelectionObjectiveFunction(theta)),
                                std::shared_ptr<numopt_common::NonlinearFunctionConstraints>(new PortfolioSelectionFunctionConstraints()))
  {

  }
  virtual ~PortfolioSelectionProblem() {}
};

} // namespace
