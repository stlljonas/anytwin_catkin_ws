/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Dario Bellicoso, Christian Gehring, Stelian Coros
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
* @file    QPFunctionMinimizer_test.cpp
* @author  Dario Bellicoso
* @date    Jan 19, 2016
*/


#include <gtest/gtest.h>
#include <numopt_common/numopt_gtest_eigen.hpp>
#include <numopt_common/numopt_math.hpp>
#include <numopt_common/ParameterizationIdentity.hpp>

#include "numopt_problems/QPExampleProblem.hpp"
#include "numopt_problems/HockSchittkowski03Problem.hpp"

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <numopt_quadprog/ActiveSetFunctionMinimizer.hpp>



TEST(QPMinimizerTest, QuadProg_minimize)
{
  constexpr int solutionDimension = 2;
  constexpr int numEq = 2;
  constexpr int numIneq = 2;

  numopt_quadprog::ActiveSetFunctionMinimizer qpMinimzer;
  numopt_common::LinearFunctionConstraints qpFunctionConstraints;
  numopt_common::QuadraticObjectiveFunction costFunction;

  numopt_common::ParameterizationIdentity x(solutionDimension);
  x.setIdentity(x.getParams());

  numopt_common::SparseMatrix Q = numopt_common::SparseMatrix(solutionDimension, solutionDimension);
  numopt_common::Vector c(solutionDimension);
  Q.setIdentity();
  c.setZero();

  numopt_common::SparseMatrix A(numEq, solutionDimension), C(numIneq, solutionDimension);
  numopt_common::Vector b(numEq), f(numIneq);

  Eigen::Matrix<double, numEq, solutionDimension> Adense;
  Adense << 1.0, 0.0,
            0.0, 1.0;
  A = Adense.sparseView();
  b.setRandom();

  Eigen::Matrix<double, numEq, solutionDimension> Cdense;
  Cdense << 1.0, 0.0,
            0.0, 1.0;
  C = Cdense.sparseView();
  f.setRandom();

  costFunction.setGlobalHessian(Q);
  costFunction.setLinearTerm(c);

  qpFunctionConstraints.setGlobalEqualityConstraintJacobian(A);
  qpFunctionConstraints.setEqualityConstraintTargetValues(b);

  numopt_common::QuadraticProblem problem = numopt_common::QuadraticProblem(std::shared_ptr<numopt_common::QuadraticObjectiveFunction>(new numopt_common::QuadraticObjectiveFunction(costFunction)),
                                                                            std::shared_ptr<numopt_common::LinearFunctionConstraints>(new numopt_common::LinearFunctionConstraints(qpFunctionConstraints)));
  double cost = 0.0;

  // solve
  qpMinimzer.minimize(&problem, x, cost);

  EXPECT_NEAR(b(0), x.getParams()(0), 1e-8);
  EXPECT_NEAR(b(1), x.getParams()(1), 1e-8);
  EXPECT_NEAR(cost, 0.5*x.getParams().transpose()*Q*x.getParams(), 1e-8);
}


TEST(QPMinimizerTest, QPExample_minimize)
{
  numopt_quadprog::ActiveSetFunctionMinimizer solver;
  numopt_problems::QPExampleProblem objective;


  numopt_common::ParameterizationIdentity p(2);
  p.getParams()(0) = 5;
  p.getParams()(1) = 4;
  double functionValue = 0;

  // solve
  ASSERT_TRUE(solver.minimize(&objective, p, functionValue));

  EXPECT_NEAR(-31.0, functionValue, 1e-3) << "Solution: " << p.getParams().transpose();
  EXPECT_NEAR(3.0, p.getParams()(0), 1e-3);
  EXPECT_NEAR(2.0, p.getParams()(1), 1e-3);

}

TEST(QPMinimizerTest, HockSchittkowski03Problem_minimize)
{
  using namespace numopt_quadprog;
  ActiveSetFunctionMinimizer solver;

  numopt_problems::HockSchittkowski03Problem objective(1.0e-3);


  numopt_common::ParameterizationIdentity p(2);
  p.getParams()(0) = 10.0;
  p.getParams()(1) = 1.0;
  double functionValue = 0;
//  ASSERT_TRUE(numopt_common::isPositiveDefinite(objective.getQuadraticObjectiveFunctionPtr()->getHessianAt(p))) << "Q is not positive definite!";

  // solve
  ASSERT_TRUE(solver.minimize(&objective, p, functionValue));

  EXPECT_NEAR(0.0, functionValue, 1e-3) << "Solution: " << p.getParams().transpose();
  EXPECT_NEAR(0.0, p.getParams()(0), 1e-3);
  EXPECT_NEAR(0.0, p.getParams()(1), 1e-3);

}


TEST(QPMinimizerTest, Unconstrained_minimize)
{
  using namespace numopt_quadprog;
  ActiveSetFunctionMinimizer solver;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2,2);
  Eigen::VectorXd c = Eigen::VectorXd::Zero(2);

  numopt_common::QuadraticObjectiveFunction costFunction;
  numopt_common::LinearFunctionConstraints functionConstraints;

  costFunction.setGlobalHessian(Q.sparseView());
  costFunction.setLinearTerm(c);

  numopt_common::QuadraticProblem problem = numopt_common::QuadraticProblem(
      std::shared_ptr<numopt_common::QuadraticObjectiveFunction>(
          new numopt_common::QuadraticObjectiveFunction(costFunction)),
      std::shared_ptr<numopt_common::LinearFunctionConstraints>(
          new numopt_common::LinearFunctionConstraints(functionConstraints)));

  numopt_common::ParameterizationIdentity p(2);
  double functionValue = 0;

  // solve
  ASSERT_TRUE(solver.minimize(&problem, p, functionValue));

  EXPECT_NEAR(0.0, functionValue, 1e-3) << "Solution: " << p.getParams().transpose();
  EXPECT_NEAR(0.0, p.getParams()(0), 1e-3);
  EXPECT_NEAR(0.0, p.getParams()(1), 1e-3);
}


TEST(QPMinimizerTest, Bounded_minimize)
{
  using namespace numopt_quadprog;
  ActiveSetFunctionMinimizer solver;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2,2);
  Eigen::VectorXd c = Eigen::VectorXd::Zero(2);

  Eigen::VectorXd u = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd l = Eigen::VectorXd::Zero(2);
  l[0] = rand() % 100 + 1;
  l[1] = rand() % 100 + 1;

  u[0] = std::numeric_limits<double>::infinity();
  u[1] = std::numeric_limits<double>::infinity();

  numopt_common::QuadraticObjectiveFunction costFunction;
  numopt_common::LinearFunctionConstraints functionConstraints;

  costFunction.setGlobalHessian(Q.sparseView());
  costFunction.setLinearTerm(c);

  functionConstraints.setGlobalBoundConstraintMaxValues(u);
  functionConstraints.setGlobalBoundConstraintMinValues(l);

  numopt_common::QuadraticProblem problem = numopt_common::QuadraticProblem(
      std::shared_ptr<numopt_common::QuadraticObjectiveFunction>(
          new numopt_common::QuadraticObjectiveFunction(costFunction)),
      std::shared_ptr<numopt_common::LinearFunctionConstraints>(
          new numopt_common::LinearFunctionConstraints(functionConstraints)));


  numopt_common::ParameterizationIdentity p(2);
  double functionValue = 0;

  // solve
  ASSERT_TRUE(solver.minimize(&problem, p, functionValue));

  double expectedFunctionValue = (0.5*l.transpose()*Q*l)(0) + (c.transpose()*l)(0);

  EXPECT_NEAR(expectedFunctionValue, functionValue, 1e-3) << "Solution: " << p.getParams().transpose();
  EXPECT_NEAR(l[0], p.getParams()[0], 1e-3);
  EXPECT_NEAR(l[1], p.getParams()[1], 1e-3);
}

TEST(QPMinimizerTest, InequalityConstraints_minimize)
{
  using namespace numopt_quadprog;
  ActiveSetFunctionMinimizer solver;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2,2);
  Eigen::VectorXd c = Eigen::VectorXd::Zero(2);

  Eigen::MatrixXd C = Eigen::MatrixXd::Identity(2,2);
  Eigen::VectorXd f = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd d = Eigen::VectorXd::Zero(2);
  d[0] = rand() % 100 + 1;
  d[1] = rand() % 100 + 1;

  f[0] = std::numeric_limits<double>::infinity();
  f[1] = std::numeric_limits<double>::infinity();

  numopt_common::QuadraticObjectiveFunction costFunction;
  numopt_common::LinearFunctionConstraints functionConstraints;

  costFunction.setGlobalHessian(Q.sparseView());
  costFunction.setLinearTerm(c);

  functionConstraints.setGlobalInequalityConstraintJacobian(C.sparseView());
  functionConstraints.setInequalityConstraintMaxValues(f);
  functionConstraints.setInequalityConstraintMinValues(d);

  numopt_common::QuadraticProblem problem = numopt_common::QuadraticProblem(
      std::shared_ptr<numopt_common::QuadraticObjectiveFunction>(
          new numopt_common::QuadraticObjectiveFunction(costFunction)),
      std::shared_ptr<numopt_common::LinearFunctionConstraints>(
          new numopt_common::LinearFunctionConstraints(functionConstraints)));

  numopt_common::ParameterizationIdentity p(2);
  double functionValue = 0;

  // solve
  ASSERT_TRUE(solver.minimize(&problem, p, functionValue));

  double expectedFunctionValue = (0.5*d.transpose()*Q*d)(0) + (c.transpose()*d)(0);

  EXPECT_NEAR(expectedFunctionValue, functionValue, 1e-3) << "Solution: " << p.getParams().transpose();
  EXPECT_NEAR(d[0], p.getParams()[0], 1e-3);
  EXPECT_NEAR(d[1], p.getParams()[1], 1e-3);
}

TEST(QPMinimizerTest, QP_with_OffDiagonal_HessianElements)
{
  using namespace numopt_quadprog;
  ActiveSetFunctionMinimizer solver;

  const int numParam = 3;
  const int numIneq = 1;

  Eigen::MatrixXd H(numParam, numParam);
  H << 1.0, 0.1, 0.0,
       0.1, 1.0, 0.0,
       0.0, 0.0, 1.0;
  Eigen::MatrixXd D(numIneq, numParam);
  D << 10.0, 0.0, 0.0;

  // solution and langrange multipliers
  const Eigen::VectorXd x = Eigen::VectorXd::Ones(numParam);
  const Eigen::VectorXd v = Eigen::VectorXd::Ones(numIneq);  // All constraints are active

  // Construct problem based on KKT conditions at the solution
  const Eigen::VectorXd c = -(H * x + D.transpose() * v);
  const Eigen::VectorXd f = D * x;
  const Eigen::VectorXd d = -std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(numIneq);

  // Construct cost
  numopt_common::QuadraticObjectiveFunction costFunction;
  costFunction.setGlobalHessian(H.sparseView());
  costFunction.setLinearTerm(c);

  // Construct constraints
  numopt_common::LinearFunctionConstraints functionConstraints;
  functionConstraints.setGlobalInequalityConstraintJacobian(D.sparseView());
  functionConstraints.setInequalityConstraintMaxValues(f);
  functionConstraints.setInequalityConstraintMinValues(d);

  // Construct problem
  numopt_common::QuadraticProblem problem = numopt_common::QuadraticProblem(
      std::shared_ptr<numopt_common::QuadraticObjectiveFunction>(new numopt_common::QuadraticObjectiveFunction(costFunction)),
      std::shared_ptr<numopt_common::LinearFunctionConstraints>(new numopt_common::LinearFunctionConstraints(functionConstraints)));

  // solve
  numopt_common::ParameterizationIdentity p(numParam);
  double functionValue = 0;
  ASSERT_TRUE(solver.minimize(&problem, p, functionValue));

  const Eigen::VectorXd& expectedSolution = x;
  const double expectedFunctionValue = (0.5 * x.transpose() * H * x)(0) + (c.transpose() * x)(0);

  EXPECT_NEAR(expectedFunctionValue, functionValue, 1e-6) << "Solution: " << p.getParams().transpose() << "\n"
                                                          << "Constraint: " << D * x - f << std::endl;
  for (int i = 0; i < numParam; i++) {
    EXPECT_NEAR(expectedSolution[0], p.getParams()[0], 1e-6);
  }
}


