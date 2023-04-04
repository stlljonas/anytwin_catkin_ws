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

#include "numopt_problems/QPExampleProblem.hpp"

TEST(QPExampleProblem, objective)
{
  using namespace numopt_problems;
  using namespace numopt_common;
  ParameterizationIdentity params(2);
  QPExampleObjectiveFunction objective;
  test_objective_function_gradient(&objective, "QuadObjectiveFunction", params);
  test_objective_function_hessian(&objective, "QuadObjectiveFunction", params);
}

TEST(QPExampleProblem, constraints_jacobians)
{
  using namespace numopt_problems;
  using namespace numopt_common;
  ParameterizationIdentity params(2);
  QPExampleFunctionConstraints constraints;
  test_function_constraints_inequality_jacobian(&constraints, "FunctionConstraintsExample1", params);
  test_function_constraints_equality_jacobian(&constraints, "FunctionConstraintsExample1", params);
}

TEST(QPExampleProblem, constraints_hessians)
{
  using namespace numopt_problems;
  using namespace numopt_common;
  ParameterizationIdentity params(2);
  QPExampleFunctionConstraints constraints;
  test_function_constraints_inequality_hessians(&constraints, "FunctionConstraintsExample1", params);
  test_function_constraints_equality_hessians(&constraints, "FunctionConstraintsExample1", params);
}
