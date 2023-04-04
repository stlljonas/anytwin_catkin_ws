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
* @file    ObjectiveFunction_test.cpp
* @author  Christian Gehring, Stelian Coros
* @date    Aug 16, 2015
*/


#include <gtest/gtest.h>
#include <numopt_common/numopt_gtest_eigen.hpp>
#include <numopt_common/NonlinearObjectiveFunction.hpp>
#include <numopt_common/ParameterizationIdentity.hpp>

#include <Eigen/Core>
#include <Eigen/SparseCore>


namespace numopt_common {

/*! Test class that implements the objective function:
 * f(x,y) = x^3 + 3*y^3 + x*y + 3*x + 2
 */
class ExampleObjectiveFunction: public NonlinearObjectiveFunction {
public:
  ExampleObjectiveFunction():NonlinearObjectiveFunction() {

  }
  virtual ~ExampleObjectiveFunction() {

  }

  virtual bool computeValue(Scalar& value, const Parameterization& params, bool newParams) {
    // f(x,y) = x^3 + 3*y^3 + x*y + 3*x + 2
    auto&  p= params.getParams();
    value =  p(0)*p(0)*p(0) + 3*p(1)*p(1)*p(1) + p(0)*p(1) + 3*p(0) + 2;
    return true;
  }

  virtual bool getLocalGradient(Vector& gradient, const Parameterization& params, bool newParams) {
    // J = [3*x^2 + y + 3, 9*y^2 + x]
    auto&  p = params.getParams();
    gradient.resize(2);
    gradient(0) = 3*p(0)*p(0) + p(1) + 3;
    gradient(1) = 9*p(1)*p(1) + p(0);
    return true;
  }

  virtual bool getLocalHessian(SparseMatrix& hessian, const Parameterization& params, bool newParams) {
    SMTriplets triplets;
    if (!getLocalHessianTriplets(triplets, params, newParams)) {
      return false;
    }
    hessian.resize(params.getLocalSize(), params.getLocalSize());
    hessian.setFromTriplets(triplets.begin(), triplets.end());
    return true;
  }

  virtual bool getLocalHessianTriplets(SMTriplets& triplets, const Parameterization& params, bool newParams) {
    // H = [6*x,   1
    //       1, 18*y]
    auto& p = params.getParams();
    triplets.clear();
    triplets.push_back(SMTriplet(0,0, 6*p(0)));
    triplets.push_back(SMTriplet(0,1, 1));
    triplets.push_back(SMTriplet(1,0, 1));
    triplets.push_back(SMTriplet(1,1, 18*p(1)));
    return true;
  }

};


} // namespace numopt_common



class NonlinearObjectiveFunctionTest : public testing::Test  {
 public:
  NonlinearObjectiveFunctionTest():
    objective(),
    params(2)
 {
    // test sample:
    params.getParams()(0) = -1;
    params.getParams()(1) = 1;
  }
  numopt_common::ExampleObjectiveFunction objective;
  numopt_common::ParameterizationIdentity params;
};


TEST_F(NonlinearObjectiveFunctionTest, computeValue)
{
  using namespace numopt_common;
  double value = 4;
  ASSERT_TRUE(objective.computeValue(value, params, true));
  EXPECT_EQ(0.0, value);
}

TEST_F(NonlinearObjectiveFunctionTest, estimateLocalGradient)
{
  using namespace numopt_common;

  Vector estimatedGradient;
  Vector trueGradient;
  ASSERT_TRUE(objective.getLocalGradient(trueGradient, params, true));
  ASSERT_TRUE(objective.estimateLocalGradient(estimatedGradient,params));
  NUMOPT_ASSERT_DOUBLE_MX_EQ(trueGradient, estimatedGradient, 1e-3, "Jacobian");

}


TEST_F(NonlinearObjectiveFunctionTest, estimateLocalHessian)
{
  using namespace numopt_common;
  SparseMatrix estimatedHessian;
  SparseMatrix trueHessian;
  ASSERT_TRUE(objective.getLocalHessian(trueHessian, params, true));
  ASSERT_TRUE(objective.estimateLocalHessian(estimatedHessian,params));
  NUMOPT_ASSERT_DOUBLE_SPARSE_MX_EQ(trueHessian, estimatedHessian, 1e-3, "Hessian");
}


