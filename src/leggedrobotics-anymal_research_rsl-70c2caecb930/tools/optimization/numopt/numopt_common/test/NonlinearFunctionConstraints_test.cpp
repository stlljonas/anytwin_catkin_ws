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
* @file    FunctionConstraints_test.cpp
* @author  Christian Gehring, Stelian Coros
* @date    Aug 16, 2015
*/


#include <gtest/gtest.h>
#include <numopt_common/numopt_gtest_eigen.hpp>
#include <numopt_common/NonlinearFunctionConstraints.hpp>
#include <numopt_common/ParameterizationIdentity.hpp>

#include <Eigen/Core>
#include <Eigen/SparseCore>

namespace numopt_common {

/*! Test class that implements the constraints:
 * x^2 + y = 0 and
 * -x*y - 10 <= 0
 */
class TestFunctionConstraints: public NonlinearFunctionConstraints {
public:
  TestFunctionConstraints():NonlinearFunctionConstraints() {
    nEqualityConstraints_ = 1;
    nInequalityConstraints_ = 1;
  }
  virtual ~TestFunctionConstraints() {

  }

  virtual bool getGlobalBoundConstraintMinValues(Vector& values) {
    values = -std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(2);
    return true;
  }

  virtual bool getGlobalBoundConstraintMaxValues(Vector& values) {
    values = std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(2);
    return true;
  }

  virtual bool getEqualityConstraintValues(Vector& values, const Parameterization& params, bool newParams = true) {
    auto& p = params.getParams();
    values.resize(1);
    // A = x^2 + y;
    values(0) = p(0)*p(0) + p(1);
    return true;
  }

  virtual bool getEqualityConstraintTargetValues(Vector& values, const Parameterization& params, bool newParams = true) {
    values.setZero(1);
    return true;
  }

  virtual bool getLocalEqualityConstraintsJacobian(SparseMatrix& jacobian, const Parameterization& params, bool newParams = true) {
    auto& p = params.getParams();
    //  dAdp =  [ 2*x, 1]
    jacobian.resize(1,2);
    jacobian.coeffRef(0, 0) = p(0)*2;
    jacobian.coeffRef(0, 1) = 1;
    return true;
  }

  virtual bool getInequalityConstraintValues(Vector& values, const Parameterization& params, bool newParams = true) {
    auto& p = params.getParams();
    // C = -x*y - 10;
    values.resize(1);
    values(0) = -p(0)*p(1) - 10;
    return true;
  }

  virtual bool getLocalInequalityConstraintsJacobian(SparseMatrix& jacobian, const Parameterization& params, bool newParams = true) {
    auto& p = params.getParams();
    // dCdp = [ -y, -x]
    jacobian.resize(1,2);
    jacobian.coeffRef(0, 0) = -p(1);
    jacobian.coeffRef(0, 1) = -p(0);
    return true;
  }

  virtual bool getInequalityConstraintMinValues(Vector& d) {
    d = -std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(1);
    return true;
  }


  virtual bool getInequalityConstraintMaxValues(Vector& f) {
    f.setZero(1);
    return true;
  }
};


} // namespace numopt_common

class NonlinearFunctionContraintsTest : public testing::Test  {
 public:
  NonlinearFunctionContraintsTest():
    constraints(),
    params(2)
  {
    auto& p = params.getParams();
    // test sample:
    p(0) = -1;
    p(1) = 1;

  }
  numopt_common::TestFunctionConstraints constraints;
  numopt_common::ParameterizationIdentity params;
};


TEST_F(NonlinearFunctionContraintsTest, estimateEqualityConstraintsJacobian)
{
  using namespace numopt_common;
  SparseMatrix estimatedJacobian;
  SparseMatrix trueEqualityConstraintsJacobian;
  ASSERT_TRUE(constraints.getLocalEqualityConstraintsJacobian(trueEqualityConstraintsJacobian, params));
  ASSERT_TRUE(constraints.estimateLocalEqualityConstraintJacobian(estimatedJacobian, params));
  NUMOPT_ASSERT_DOUBLE_SPARSE_MX_EQ(trueEqualityConstraintsJacobian, estimatedJacobian, 1e-3, "equality constraints jacobian");
}

TEST_F(NonlinearFunctionContraintsTest, estimateInEqualityConstraintsJacobian)
{
  using namespace numopt_common;
  SparseMatrix estimatedJacobian;
  SparseMatrix trueInequalityConstraintsJacobian;
  ASSERT_TRUE(constraints.getLocalInequalityConstraintsJacobian(trueInequalityConstraintsJacobian, params));
  ASSERT_TRUE(constraints.estimateLocalInequalityConstraintJacobian(estimatedJacobian, params));
  NUMOPT_ASSERT_DOUBLE_SPARSE_MX_EQ(trueInequalityConstraintsJacobian, estimatedJacobian, 1e-3, "inequality constraints jacobian");
}
