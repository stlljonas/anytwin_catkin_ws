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
* @file    L2Regularizer_test.cpp
* @author  Christian Gehring
* @date    June, 2016
*/


#include <gtest/gtest.h>
#include <numopt_common/numopt_gtest.hpp>
#include <numopt_common/L2Regularizer.hpp>
#include <numopt_common/ParameterizationIdentity.hpp>

#include <Eigen/Core>
#include <Eigen/SparseCore>


TEST(L2RegularizerTest, computeValue)
{
  using namespace numopt_common;
  ParameterizationIdentity params(2);
  params.getParams()(0) = 5.0;
  params.getParams()(1) = 10.0;

  L2Regularizer objective(2, 4.0);

  double value = 4;
  ASSERT_TRUE(objective.computeValue(value, params, true));

  double expectedValue = 0.5*4.0*(5.0*5.0 + 10.0*10.0);
  EXPECT_EQ(expectedValue, value);
}

TEST(L2RegularizerTest, estimateLocalGradient)
{
  using namespace numopt_common;
  ParameterizationIdentity params(2);
  L2Regularizer objective(2, 4.0);
  test_objective_function_gradient(&objective, "gradient", params);
}


TEST(L2RegularizerTest, estimateLocalHessian)
{
  using namespace numopt_common;
  ParameterizationIdentity params(2);
  L2Regularizer objective(2, 4.0);
  test_objective_function_hessian(&objective, "hessian", params);
}


