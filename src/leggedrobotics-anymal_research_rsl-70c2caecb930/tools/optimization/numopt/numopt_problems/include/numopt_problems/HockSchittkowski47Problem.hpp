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
* @file    HockSchittkowski47Problem.hpp
* @author  Anna-Maria Georgarakis, Christian Gehring
* @date    Oct. 11 2015
*/
#pragma once

#include "numopt_common/ConstrainedNonlinearProblem.hpp"

#include "numopt_common/NonlinearObjectiveFunction.hpp"
#include "numopt_common/NonlinearFunctionConstraints.hpp"


namespace numopt_problems {


/*!
 * Hock Schittkowski Collection, Problem 47
 * *
 * f(x1,x2,x3,x4,x5) = (x1-x2)² + (x2-x3)³ + (x3-x4)⁴ + (x4-x5)⁴
 *  s.t.   x1 + x2² + x3³ - 3 = 0
 *          x2 - x3³ + x4 - 1 = 0
 *                  x1*x5 - 1 = 0
 *
 * 5D non-convex non-linear function with polynomial equality constraints.
 *
 * Start: x0 = [2, sqrt(2), -1, 2-sqrt(2), 0.5] (feasible)
 *        f(x0) = 20.7381   // Note that the value in the reference is wrong.
 *
 * Solution: x* = [1, 1, 1, 1, 1]
 *            f(x*) = 0.0
 * Reference:
 *  W. Hock and K. Schittkowski, “Test Examples for Nonlinear Programming
 *  Codes,” Lecture Notes in Economics and Mathematical Systems, vol. 187,
 *  1981,2009.
 *
 */
class HS47ObjectiveFunction : public numopt_common::NonlinearObjectiveFunction {

 public:
  HS47ObjectiveFunction():NonlinearObjectiveFunction(){


  }

  virtual ~HS47ObjectiveFunction() {}

  virtual bool computeValue(numopt_common::Scalar& value, const numopt_common::Parameterization& params, bool newParams = true) {
    //return (x1-x2)²+(x2-x3)³+(x3-x4)⁴+(x4-x5)⁴;

    auto& p = params.getParams();
    double diff1 = p(0)-p(1);
    double diff2 = p(1)-p(2);
    double diff3 = p(2)-p(3);
    double diff4 = p(3)-p(4);
    value =  pow(diff1,2.0)+pow(diff2,3.0)+pow(diff3,4.0)+pow(diff4,4.0);
    return true;
  }

  virtual bool getLocalGradient(numopt_common::Vector& gradient, const numopt_common::Parameterization& params, bool newParams = true) {
    /* J = [2(x1-x2) , -2(x1-x2)+3(x2-x3)² , -3(x2-x3)²+4(x3-x4)³ , -4(x3-x4)³+4(x4-x5)³ , -4(x4-x5)³]
     * */
    gradient.resize(5);
    auto& p = params.getParams();
    double diff1 = p(0)-p(1);
    double diff2 = p(1)-p(2);
    double diff3 = p(2)-p(3);
    double diff4 = p(3)-p(4);
    gradient(0) =  2.0*diff1;
    gradient(1) = -2.0*diff1 + 3.0*diff2*diff2;
    gradient(2) = -3.0*diff2*diff2 + 4.0*pow(diff3,3.0);
    gradient(3) =  -4.0*pow(diff3,3.0) + 4.0*pow(diff4,3.0);
    gradient(4) =  -4.0*pow(diff4,3.0);
    return true;
  }

  virtual bool getLocalHessian(numopt_common::SparseMatrix& hessian, const numopt_common::Parameterization& params, bool newParams = true) {
    /*  H =[ 2  -2           0                    0                      0         ]
    *      [-2   2+6(x2-x3) -6(x2-x3)             0                      0         ]
    *      [ 0  -6(x2-x3)    6(x2-x3)+12(x3-x4)²  -12(x3-x4)²             0         ]
    *      [ 0   0           -12(x3-x4)²          12(x3-x4)²+12(x4-x5)²  -12(x4-x5)²]
    *      [ 0   0           0                    -12(x4-x5)²            12(x4-x5)²]
    */
    hessian.resize(5,5);
    auto& p = params.getParams();
    double diff2 = p(1)-p(2);
    double diff3 = p(2)-p(3);
    double diff4 = p(3)-p(4);
    hessian.setZero();
    hessian.coeffRef(0,0) = 2.0;
    hessian.coeffRef(1,1) = 2.0+6.0*diff2;
    hessian.coeffRef(2,2) = 6.0*diff2+12.0*diff3*diff3;
    hessian.coeffRef(3,3) = 12.0*diff3*diff3+12.0*diff4*diff4;
    hessian.coeffRef(4,4) =  12.0*diff4*diff4;
    hessian.coeffRef(1,0) = -2.0;
    hessian.coeffRef(0,1) = -2.0;
    hessian.coeffRef(2,1) = -6.0*diff2;
    hessian.coeffRef(1,2) = -6.0*diff2;
    hessian.coeffRef(3,2) =  -12.0*diff3*diff3;
    hessian.coeffRef(2,3) =  -12.0*diff3*diff3;
    hessian.coeffRef(4,3) =  -12.0*diff4*diff4;
    hessian.coeffRef(3,4) =  -12.0*diff4*diff4;
    return true;
  }

};


/*!
 * polynomial constraints
 * x1 + x2² + x3³ - 3 = 0
 *  x2 - x3² + x4 - 1 = 0
 *          x1*x5 - 1 = 0
 *
 * dAdp = [1   2*x2  3*x3²  0  0
 *         0   1    -2*x3   1  0
 *         x5  0     0      0  x1]
 *
 * b = [0,0,0]
 */
class HS47FunctionConstraints: public numopt_common::NonlinearFunctionConstraints {
public:
  HS47FunctionConstraints():NonlinearFunctionConstraints() {
    // equality constraints
    nEqualityConstraints_ = 3;

    // inequality constraints
    nInequalityConstraints_ = 0;

  }
  virtual ~HS47FunctionConstraints() {

  }

  virtual bool getLocalEqualityConstraintJacobian(numopt_common::SparseMatrix& jacobian, const numopt_common::Parameterization& params, bool newParams = true) {
    //dAdp.coeffRef.setZero();
    jacobian.resize(nEqualityConstraints_,5);
    auto& p = params.getParams();
    jacobian.coeffRef(0,0) = 1.0;
    jacobian.coeffRef(0,1) = 2.0*p(1);
    jacobian.coeffRef(0,2) = 3.0*p(2)*p(2);
    jacobian.coeffRef(1,1) = 1.0;
    jacobian.coeffRef(1,2) = -2.0*p(2);
    jacobian.coeffRef(1,3) = 1.0;
    jacobian.coeffRef(2,0) = p(4);
    jacobian.coeffRef(2,4) = p(0);
    return true;
  }


  virtual bool getLocalEqualityConstraintHessian(numopt_common::SparseMatrix& hessian, numopt_common::Parameterization& params, int iConstraint, bool newParams = true) {
    auto& p = params.getParams();
    hessian.resize(5, 5);

//    * x1 + x2² + x3³ - 3 = 0
//    *  x2 - x3² + x4 - 1 = 0
//    *          x1*x5 - 1 = 0

    switch (iConstraint) {
      case (0):
      // J = [1 2*p(1) 3*p(2)^2 0 0 0]

//       * H = [0 0 0 0 0 0
//              0 2 0 0 0 0
//              0 0 6*p(2) 0 0
//              0 0 0 0 0
//              0 0 0 0 0]

      hessian.coeffRef(1,1) = 2.0;
      hessian.coeffRef(2,2) = 6.0*p(2);
      break;
      case (1):
        // J = [0 1 -2.0*p(2) 1.0 0 0]
//         H = [0 0 0 0 0
//         *      0 0 0 0 0
//         *      0 0 -2 0 0
//         *      0 0 0 0 0
//         *      0 0 0 0 0]

          hessian.coeffRef(2,2) = -2.0;
      break;
      case (2):
           // J = [p(4) 0 0 0 p(0)]
//             H = [0 0 0 0 1
//             *      0 0 0 0 0
//             *      0 0 0 0 0
//             *      0 0 0 0 0
//             *      1 0 0 0 0]

            hessian.coeffRef(0,4) = 1.0;
            hessian.coeffRef(4,0) = 1.0;
          break;
      default:
        ;
    }
    return true;
  }


  virtual bool getEqualityConstraintValues(numopt_common::Vector& values, const numopt_common::Parameterization& params, bool newParams = true) {
    auto& p = params.getParams();
    values.resize(3);
    values(0) = p(0)+p(1)*p(1)+p(2)*p(2)*p(2)-3.0;
    values(1) = p(1)-p(2)*p(2)+p(3)-1.0;
    values(2) = p(0)*p(4)-1.0;
    return true;
  }

  virtual bool getEqualityConstraintTargetValues(numopt_common::Vector& values) {
    values.setZero(3);
    return true;
  }

  virtual bool getGlobalBoundConstraintMinValues(numopt_common::Vector& values) {
    values = -std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(5);
    return true;
  }

  virtual bool getGlobalBoundConstraintMaxValues(numopt_common::Vector& values) {
    values = std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(5);
    return true;
  }

};


class HockSchittkowski47Problem : public numopt_common::ConstrainedNonlinearProblem {
 public:
  HockSchittkowski47Problem():ConstrainedNonlinearProblem(std::shared_ptr<numopt_common::NonlinearObjectiveFunction>(new HS47ObjectiveFunction),
                                                          std::shared_ptr<numopt_common::NonlinearFunctionConstraints>(new HS47FunctionConstraints())) {

    }

  virtual ~HockSchittkowski47Problem() {}
};

} // namespace
