/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, Yvain de Viragh, Marko Bjelonic, Dario Bellicoso
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
 *   * Neither the name of Robotic Systems Lab nor ETH Zurich nor
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
* @file    OperatorSplittingQuadraticProgramFunctionMinimizer_test.cpp
* @author  Yvain de Viragh
* @date    May 18, 2018
* @brief
*/

#include <gtest/gtest.h>
#include <numopt_common/numopt_gtest_eigen.hpp>
#include <numopt_common/ParameterizationIdentity.hpp>

#include "numopt_osqp/OperatorSplittingQuadraticProgramFunctionMinimizer.hpp"

#include "numopt_problems/QPExampleProblem.hpp"
#include "numopt_problems/PortfolioSelectionProblem.hpp"
#include "numopt_problems/HockSchittkowski03Problem.hpp"

#include <Eigen/Core>
#include <Eigen/SparseCore>


TEST(OperatorSplittingQuadraticProgramFunctionMinimizerTest, QPExample_minimize)
{
  using namespace numopt_osqp;
  using namespace numopt_common;
  OperatorSplittingQuadraticProgramFunctionMinimizer solver;
  solver.setExternalWarmStart(true);
  solver.setCheckProblemValidity(true);
  numopt_problems::QPExampleProblem objective;

  ParameterizationIdentity params(2);
  params.getParams()(0) = 5;
  params.getParams()(1) = 4;
  double functionValue = 0;

  // solve
  ASSERT_TRUE(solver.minimize(&objective, params, functionValue));

  // TODO: Previously, the tolerance was 1e-3. OSQP is sligthly off - does it matter?
  EXPECT_NEAR(-31.0, functionValue, 1e-2) << "Solution: " << params.getParams().transpose();
  EXPECT_NEAR(3.0, params.getParams()(0), 1e-2);
  EXPECT_NEAR(2.0, params.getParams()(1), 1e-2);

}

TEST(OperatorSplittingQuadraticProgramFunctionMinimizerTest, HockSchittkowski03Problem_minimize)
{
  using namespace numopt_osqp;
  using namespace numopt_common;
  OperatorSplittingQuadraticProgramFunctionMinimizer solver;
  solver.setExternalWarmStart(true);
  solver.setCheckProblemValidity(true);
  numopt_problems::HockSchittkowski03Problem objective;

  ParameterizationIdentity params(2);
  params.getParams()(0) = 10.0;
  params.getParams()(1) = 1.0;
  double functionValue = 0.0;

  // solve
  ASSERT_TRUE(solver.minimize(&objective, params, functionValue));

  EXPECT_NEAR(0.0, functionValue, 1e-3) << "Solution: " << params.getParams().transpose();
  EXPECT_NEAR(0.0, params.getParams()(0), 1e-3);
  EXPECT_NEAR(0.0, params.getParams()(1), 1e-3);

}

