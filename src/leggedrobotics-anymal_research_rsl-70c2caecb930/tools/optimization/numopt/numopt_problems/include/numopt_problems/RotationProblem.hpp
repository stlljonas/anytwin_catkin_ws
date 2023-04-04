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
* @file    RotationProblem
* @author  Christian Gehring
* @date    June, 2016
*/

#pragma once

#include <kindr/Core>
#include "numopt_common/ConstrainedNonlinearProblem.hpp"

#include "numopt_common/NonlinearObjectiveFunction.hpp"
#include "numopt_common/NonlinearFunctionConstraints.hpp"


namespace numopt_problems {


/*!
 *
 * f(q) = 0.5 * || q_IB - qDes_IB ||^2
 *
 * global coordinates: q = q_BI
 * local coordinates:  u = I_w_IB
 *
 */
class RotationObjectiveFunction : public numopt_common::NonlinearObjectiveFunction {

 public:
  typedef kindr::RotationQuaternionD Quaternion;
  typedef kindr::EulerAnglesZyxD EulerAnglesZyx;
 protected:
  Quaternion q_BI_des_;
 public:
  RotationObjectiveFunction(const Quaternion& desRotation = Quaternion(EulerAnglesZyx(M_PI/2.0, 0.0, 0.0))):NonlinearObjectiveFunction(),
    q_BI_des_(desRotation)
  {

  }

  virtual ~RotationObjectiveFunction() {}


  virtual bool computeValue(numopt_common::Scalar& value, const numopt_common::Parameterization& params, bool newParams = true) {
    value = Quaternion(params.getParams()).boxMinus(q_BI_des_).squaredNorm()*0.5;
    return true;
  }

  virtual bool getLocalGradient(numopt_common::Vector& gradient, const numopt_common::Parameterization& params, bool newParams = true) {

    gradient.resize(3);
    const Eigen::Vector3d I_v = Quaternion(params.getParams()).boxMinus(q_BI_des_);
    gradient = I_v;
    return true;
  }

  virtual bool getLocalHessian(numopt_common::SparseMatrix& hessian, const numopt_common::Parameterization& params, bool newParams = true) {
    hessian.resize(3,3);
    const Eigen::Vector3d I_v = Quaternion(params.getParams()).boxMinus(q_BI_des_);
    const Eigen::Matrix3d jacobian = kindr::getJacobianOfExponentialMap(I_v).inverse();
    hessian = jacobian.sparseView();
    return true;
  }

};


/*!
 *
 */
class RotationConstraints: public numopt_common::NonlinearFunctionConstraints {
public:
  RotationConstraints():NonlinearFunctionConstraints() {
    // equality constraints
    nEqualityConstraints_ = 0;

    // inequality constraints
    nInequalityConstraints_ = 0;

  }
  virtual ~RotationConstraints() {

  }

  virtual bool getGlobalBoundConstraintMinValues(numopt_common::Vector& values) {
    values = -std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(4);
    return true;
  }

  virtual bool getGlobalBoundConstraintMaxValues(numopt_common::Vector& values) {
    values = std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(4);
    return true;
  }

};


class RotationProblem : public numopt_common::ConstrainedNonlinearProblem {
 public:
  RotationProblem():ConstrainedNonlinearProblem(std::shared_ptr<numopt_common::NonlinearObjectiveFunction>(new RotationObjectiveFunction),
                                                          std::shared_ptr<numopt_common::NonlinearFunctionConstraints>(new RotationConstraints())) {

    }

  virtual ~RotationProblem() {}
};

} // namespace
