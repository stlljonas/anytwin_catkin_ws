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
* @file    SQPFunctionMinimize_test.cpp
* @author  Christian Gehring, Stelian Coros
* @date    Aug 16, 2015
*/

#include <gtest/gtest.h>
#include <numopt_common/numopt_gtest.hpp>
#include <numopt_common/ParameterizationIdentity.hpp>
#include "numopt_problems/ParameterizationQuaternion.hpp"
#include "numopt_problems/RotationProblem.hpp"

TEST(RotationProblem, objective_gradient_optimal)
{
  using namespace numopt_problems;
  using namespace numopt_common;
  typedef ParameterizationQuaternion::RotationQuaternion Quaternion;
  typedef kindr::EulerAnglesZyxPD EulerAnglesZyx;

  RotationObjectiveFunction objective;
  ParameterizationQuaternion params;

  params.getParams() = Quaternion(EulerAnglesZyx(M_PI/2.0, 0.0, 0.0)).vector();

  double functionValue = 99.0;
  ASSERT_TRUE(objective.computeValue(functionValue, params));
  EXPECT_EQ(0.0, functionValue);

  Vector estimatedLocalGradient;
  Vector expectedGradient = Vector::Zero(3);
  ASSERT_TRUE(objective.estimateLocalGradient(estimatedLocalGradient, params));
  NUMOPT_ASSERT_DOUBLE_MX_EQ(expectedGradient, estimatedLocalGradient, 1e-3, "estimatedLocalGradient");
  std::cout << "est. local gradient: " << estimatedLocalGradient.transpose() << std::endl;

  Vector estimatedGlobalGradient;
  Vector expectedGlobalGradient = Vector::Zero(4);
  ASSERT_TRUE(params.transformLocalVectorToGlobalVector(estimatedGlobalGradient, params.getParams(), estimatedLocalGradient));
  NUMOPT_ASSERT_DOUBLE_MX_EQ(expectedGlobalGradient, estimatedGlobalGradient, 1e-3, "estimatedGlobalGradient");
  std::cout << "est. global gradient: " << estimatedGlobalGradient.transpose() << std::endl;

  Vector localGradient;
  ASSERT_TRUE(objective.getLocalGradient(localGradient, params));
  NUMOPT_ASSERT_DOUBLE_MX_EQ(expectedGradient, localGradient, 1e-3, "localGradient");
  std::cout << "anal. local gradient: " << localGradient.transpose() << std::endl;
}

TEST(RotationProblem, objective_gradient_init)
{
  using namespace numopt_problems;
  using namespace numopt_common;
  typedef ParameterizationQuaternion::RotationQuaternion Quaternion;
  typedef kindr::EulerAnglesZyxPD EulerAnglesZyx;

  RotationObjectiveFunction objective;
  ParameterizationQuaternion params;

  params.getParams() = Quaternion(EulerAnglesZyx(0.0, 0.0, 0.0)).vector();

  double functionValue = 99.0;
  ASSERT_TRUE(objective.computeValue(functionValue, params));
  EXPECT_EQ((M_PI/2.0)*(M_PI/2.0)*0.5, functionValue);

  Vector estimatedLocalGradient;
  Vector expectedGradient = Vector::Zero(3);
  expectedGradient(2) = -(M_PI/2.0);
  ASSERT_TRUE(objective.estimateLocalGradient(estimatedLocalGradient, params));
  NUMOPT_ASSERT_DOUBLE_MX_EQ(expectedGradient, estimatedLocalGradient, 1e-3, "estimatedLocalGradient");
  std::cout << "est. local gradient: " << estimatedLocalGradient.transpose() << std::endl;

  Vector estimatedGlobalGradient;
  Vector expectedGlobalGradient = Vector::Zero(4);
  ASSERT_TRUE(params.transformLocalVectorToGlobalVector(estimatedGlobalGradient, params.getParams(), estimatedLocalGradient));
  //ASSERT_DOUBLE_MX_EQ(expectedGlobalGradient, estimatedGlobalGradient, 1e-3, "estimatedGlobalGradient");
  std::cout << "est. global gradient: " << estimatedGlobalGradient.transpose() << std::endl;

  Vector localGradient;
  ASSERT_TRUE(objective.getLocalGradient(localGradient, params));
  NUMOPT_ASSERT_DOUBLE_MX_EQ(expectedGradient, localGradient, 1e-3, "anal. local gradient");
  std::cout << "anal. local gradient: " << localGradient.transpose() << std::endl;
}
TEST(RotationProblem, objective_gradient_other)
{
  using namespace numopt_problems;
  using namespace numopt_common;
  typedef ParameterizationQuaternion::RotationQuaternion Quaternion;
  typedef kindr::EulerAnglesZyxPD EulerAnglesZyx;

  RotationObjectiveFunction objective;
  ParameterizationQuaternion params;

  //params.getParams() = Quaternion(EulerAnglesZyx(0.0, 0.0, 0.0)).vector();
  params.setRandom(params.getParams());
  numopt_common::Vector analyticalGradient;
  ASSERT_TRUE(objective.getLocalGradient(analyticalGradient, params));
  numopt_common::Vector estimatedGradient;
  ASSERT_TRUE(objective.estimateLocalGradient(estimatedGradient, params));
  EXPECT_EQ(estimatedGradient.size(), analyticalGradient.size());
  NUMOPT_ASSERT_DOUBLE_MX_EQ(estimatedGradient, analyticalGradient, 1e-3, std::string{"localGradient"});
}

TEST(RotationProblem, objective)
{
  using namespace numopt_problems;
  using namespace numopt_common;
  ParameterizationQuaternion params;
  RotationObjectiveFunction objective;


  typedef ParameterizationQuaternion::RotationQuaternion Quaternion;
  typedef kindr::EulerAnglesZyxPD EulerAnglesZyx;


  test_objective_function_gradient(&objective, "RotationObjectiveFunction", params);
  test_objective_function_hessian(&objective, "RotationObjectiveFunction", params);
}
