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
#include <numopt_common/numopt_gtest_eigen.hpp>

#include "numopt_sqp/SQPFunctionMinimizer.hpp"

#include "numopt_problems/QPExampleProblem.hpp"
#include "numopt_problems/PortfolioSelectionProblem.hpp"
#include "numopt_problems/HockSchittkowski05Problem.hpp"
#include "numopt_problems/HockSchittkowski47Problem.hpp"

#include <numopt_quadprog/ActiveSetFunctionMinimizer.hpp>

#include <Eigen/Core>
#include <Eigen/SparseCore>


TEST(QuadprogTest, QPExample_minimize)
{
  using namespace numopt_sqp;
  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver(new numopt_quadprog::ActiveSetFunctionMinimizer);
  SQPFunctionMinimizer solver(qpSolver);
  solver.setPrintOutput(false);

  numopt_problems::QPExampleProblem objective;

  Eigen::VectorXd p(2);
  p(0) = 5;
  p(1) = 4;
  double functionValue = 0;

  // solve
  ASSERT_TRUE(solver.minimize(&objective, p, functionValue));

  EXPECT_NEAR(-31.0, functionValue, 1e-3) << "Solution: " << p.transpose();
  EXPECT_NEAR(3.0, p(0), 1e-3);
  EXPECT_NEAR(2.0, p(1), 1e-3);

}


TEST(QuadprogTest, PortfolioSelectionObj_minimize)
{
  using namespace numopt_sqp;
  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver(new numopt_quadprog::ActiveSetFunctionMinimizer);
  SQPFunctionMinimizer solver(qpSolver,
                              1000,
                              0.0001,
                              5,
                              -DBL_MAX,
                              false,
                              false);

  numopt_problems::PortfolioSelectionProblem objective(4.0/5.0);
  Eigen::VectorXd p(2);
  p(0) = 5;
  p(1) = 0;
  double functionValue = 0;


  solver.setPrintOutput(false);


  Eigen::VectorXd pTest(2);
  pTest(0) = 5;
  pTest(1) = 0;
  EXPECT_EQ(-40, objective.getObjectiveFunctionPtr()->computeValue(pTest));

  // solve
  ASSERT_TRUE(solver.minimize(&objective, p, functionValue)) << "p: " << p.transpose();


  EXPECT_NEAR(2.5, p(0), 1e-3);
  EXPECT_NEAR(2.5, p(1), 1e-3);
  EXPECT_NEAR(-55.0, functionValue, 1e-3);



}

TEST(QuadprogTest, PortfolioSelectionObj2_minimize)
{
  using namespace numopt_sqp;
  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver(new numopt_quadprog::ActiveSetFunctionMinimizer);
  SQPFunctionMinimizer solver(qpSolver,
                              1000,
      0.0001,
      5,
      DBL_MIN,
      false,
      false);

  numopt_problems::PortfolioSelectionProblem objective(8.0/5.0);
  Eigen::VectorXd p(2);

  p(0) = 100;
  p(1) = 100;
  double functionValue = 0;

  solver.setPrintOutput(false);

  // solve
  ASSERT_TRUE(solver.minimize(&objective, p, functionValue)) << "p: " << p.transpose();

  EXPECT_NEAR(3.0/2.0, p(0), 1e-3);
  EXPECT_NEAR(7.0/4.0, p(1), 1e-3);
  EXPECT_NEAR(-29.0, functionValue, 1e-3);


}

TEST(QuadprogTest, DISABLED_HockSchittkowski05Problem_minimize)
{
  using namespace numopt_sqp;
  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver(new numopt_quadprog::ActiveSetFunctionMinimizer);
  SQPFunctionMinimizer solver(qpSolver,
                              1000,
      0.0001,
      10,
      -DBL_MAX,
      false,
      false);

  numopt_problems::HockSchittkowski05Problem objective;
  Eigen::VectorXd p(2);

  // recommended initial values
  p(0) = 0.0;
  p(1) = 0.0;
  double functionValue = 100;

  solver.setPrintOutput(false);

  // solve
  ASSERT_TRUE(solver.minimize(&objective, p, functionValue)) << "p: " << p.transpose();

  EXPECT_NEAR(-M_PI/3.0 + 0.5, p(0), 1e-3);
  EXPECT_NEAR(-M_PI/3.0 - 0.5, p(1), 1e-3);
  EXPECT_NEAR(-0.5*sqrt(3.0)-M_PI/3.0, functionValue, 1e-3);

}


TEST(QuadprogTest, DISABLED_HockSchittkowski47Problem_minimize)
{
  using namespace numopt_sqp;
  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver(new numopt_quadprog::ActiveSetFunctionMinimizer);
  SQPFunctionMinimizer solver(qpSolver,
                              1000,
      0.0001,
      10,
      -DBL_MAX,
      false,
      true);

  numopt_problems::HockSchittkowski47Problem objective;
  Eigen::VectorXd p(5);

  // recommended initial values
  p(0) = 2.0;
  p(1) = sqrt(2.0);
  p(2) = -1.0;
  p(3) = 2.0 - sqrt(2.0);
  p(4) = 0.5;
  double functionValue = 100;

  solver.setPrintOutput(true);

  // solve
  ASSERT_TRUE(solver.minimize(&objective, p, functionValue)) << "p: " << p.transpose();

  EXPECT_NEAR(1.0, p(0), 1e-3);
  EXPECT_NEAR(1.0, p(1), 1e-3);
  EXPECT_NEAR(1.0, p(2), 1e-3);
  EXPECT_NEAR(1.0, p(3), 1e-3);
  EXPECT_NEAR(1.0, p(4), 1e-3);
  EXPECT_NEAR(0.0, functionValue, 1e-3);

}

TEST(QuadprogTest, RotationProblem_minimize)
{
  using namespace numopt_sqp;
  using namespace numopt_common;
  using namespace numopt_problems;

  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver(new numopt_quadprog::ActiveSetFunctionMinimizer);
  SQPFunctionMinimizer solver(qpSolver,
                              1000,
      0.0001,
      10,
      -DBL_MAX,
      true,
      true);

  kindr::RotationQuaternionD desRotation(kindr::EulerAnglesZyxD(M_PI/2.0, 0.0, 0.0));
  RotationProblem objective;
  ParameterizationQuaternion params;

  // recommended initial values
  params.setIdentity(params.getParams());
  double functionValue = 100;

  solver.setPrintOutput(true);

  // solve
  ASSERT_TRUE(solver.minimize(&objective, params, functionValue)) << "p: " << params.getParams().transpose();

  EXPECT_NEAR(0.0, functionValue, 1e-3);
  ASSERT_TRUE(kindr::RotationQuaternionD(params.getParams()).isNear(desRotation, 1.0e-4));
}
