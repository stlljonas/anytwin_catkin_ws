/*!
* @file    HierarchicalOptimizationQpNullSpaceProjectionTest.cpp
* @author  Dario Bellicoso
* @date    Jun, 2016
*/

// Unit test facility
#include <gtest/gtest.h>
#include <numopt_common/numopt_gtest.hpp>

// hierarchical optimization
#include <hierarchical_optimization/HierarchicalOptimizationQpNullSpaceProjection.hpp>

// solver active set
#include <numopt_quadprog/ActiveSetFunctionMinimizer.hpp>


TEST(HierarchicalOptimizationQpNullSpaceProjectionTest, oneEqProblem) {

  std::string unitTestName{"constrainedProblem"};

  constexpr unsigned int solutionSpaceDimension = 2;
  std::unique_ptr<numopt_quadprog::ActiveSetFunctionMinimizer> minimizer(new numopt_quadprog::ActiveSetFunctionMinimizer());
  hopt::HierarchicalOptimizationQpNullSpaceProjection solver(solutionSpaceDimension, std::move(minimizer), true);
  solver.resetOptimization();

  Eigen::MatrixXd A, D;
  Eigen::VectorXd b, d;
  unsigned int priority;
  std::string name;

  /* Setup first problem:
   *    min ||x||^2
   */
  A = Eigen::MatrixXd::Identity(solutionSpaceDimension, solutionSpaceDimension);
  b = Eigen::VectorXd::Zero(solutionSpaceDimension);
  D = Eigen::MatrixXd();
  d = Eigen::VectorXd();

  priority = 1;
  name = "prob1";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Equality);

  Eigen::VectorXd expectedSolution = Eigen::VectorXd::Zero(solutionSpaceDimension);
  Eigen::VectorXd computedSolution = Eigen::VectorXd::Zero(solutionSpaceDimension);
  if (!solver.solveOptimization(computedSolution)) {
    throw std::runtime_error("Solver failed!");
  }

  NUMOPT_ASSERT_DOUBLE_MX_EQ(expectedSolution, computedSolution, 1e-3, unitTestName);
}


TEST(HierarchicalOptimizationQpNullSpaceProjectionTest, twoEqProblem) {

  std::string unitTestName{"constrainedProblem"};

  constexpr unsigned int solutionSpaceDimension = 2;
  std::unique_ptr<numopt_quadprog::ActiveSetFunctionMinimizer> minimizer(new numopt_quadprog::ActiveSetFunctionMinimizer());
  hopt::HierarchicalOptimizationQpNullSpaceProjection solver(solutionSpaceDimension, std::move(minimizer), true);

  Eigen::MatrixXd A, D;
  Eigen::VectorXd b, d;
  unsigned int priority;
  std::string name;

  /* Setup first problem:
   *    x1 + x2 >= 1
   */
  A = Eigen::MatrixXd();
  b = Eigen::VectorXd();
  D.setZero(1,solutionSpaceDimension);
  d.setZero(1);

  D.setOnes(1, solutionSpaceDimension);
  D *= -1.0;
  d << -1;
  priority = 1;
  name = "prob1";

  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Inequality);


  /* Setup second problem:
   *  min ||x||^2
   */
  A.setIdentity(solutionSpaceDimension, solutionSpaceDimension);
  b.setZero(solutionSpaceDimension);
  D = Eigen::MatrixXd();
  d = Eigen::VectorXd();

  priority = 2;
  name = "prob2";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Equality);

  Eigen::VectorXd expectedSolution(solutionSpaceDimension);
  expectedSolution << 0.5, 0.5;

  Eigen::VectorXd computedSolution = Eigen::VectorXd::Zero(solutionSpaceDimension);
  solver.solveOptimization(computedSolution);

  NUMOPT_ASSERT_DOUBLE_MX_EQ(expectedSolution, computedSolution, 1e-3, unitTestName);
}

TEST(HierarchicalOptimizationQpNullSpaceProjectionTest, threeEqProblem) {

  std::string unitTestName{"constrainedProblem"};

  constexpr unsigned int solutionSpaceDimension = 2;
  std::unique_ptr<numopt_quadprog::ActiveSetFunctionMinimizer> minimizer(new numopt_quadprog::ActiveSetFunctionMinimizer());
  hopt::HierarchicalOptimizationQpNullSpaceProjection solver(solutionSpaceDimension, std::move(minimizer), true);

  Eigen::MatrixXd A, D;
  Eigen::VectorXd b, d;
  unsigned int priority;
  std::string name;

  /* Setup first problem:
   *    x1 + x2 >= 1
   */
  A = Eigen::MatrixXd();
  b = Eigen::VectorXd();
  D.setZero(1,solutionSpaceDimension);
  d.setZero(1);

  D.setOnes(1, solutionSpaceDimension);
  D *= -1.0;
  d << -1;
  priority = 1;
  name = "prob1";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Inequality);

  /* Setup second problem:
   *  -3*x1 + x2 = 2
   */
  A = Eigen::MatrixXd(1,solutionSpaceDimension);
  b = Eigen::VectorXd(1);
  D = Eigen::MatrixXd();
  d = Eigen::VectorXd();
  A << -3, 1;
  b << 2;
  priority = 2;
  name = "prob2";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Equality);

  /* Setup third problem:
   *  -4*x1 + x2 = -1
   */
  A = Eigen::MatrixXd(1,solutionSpaceDimension);
  b = Eigen::VectorXd(1);
  D = Eigen::MatrixXd();
  d = Eigen::VectorXd();
  A << -4, 1;
  b << -1;
  priority = 3;
  name = "prob3";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Equality);

  Eigen::VectorXd expectedSolution(solutionSpaceDimension);
  expectedSolution << 3.0, 11.0;

  Eigen::VectorXd computedSolution = Eigen::VectorXd::Zero(solutionSpaceDimension);
  solver.solveOptimization(computedSolution);

  NUMOPT_ASSERT_DOUBLE_MX_EQ(expectedSolution, computedSolution, 1e-3, unitTestName);
}


TEST(HierarchicalOptimizationQpNullSpaceProjectionTest, fourEqProblem) {

  std::string unitTestName{"constrainedProblem"};

  constexpr unsigned int solutionSpaceDimension = 2;
  std::unique_ptr<numopt_quadprog::ActiveSetFunctionMinimizer> minimizer(new numopt_quadprog::ActiveSetFunctionMinimizer());
  hopt::HierarchicalOptimizationQpNullSpaceProjection solver(solutionSpaceDimension, std::move(minimizer), true);

  Eigen::MatrixXd A, D;
  Eigen::VectorXd b, d;
  unsigned int priority;
  std::string name;

  /* Setup first problem:
   *    x1 + x2 >= 1
   */
  A = Eigen::MatrixXd();
  b = Eigen::VectorXd();
  D.setZero(1,solutionSpaceDimension);
  d.setZero(1);

  D.setOnes(1, solutionSpaceDimension);
  D *= -1.0;
  d << -1;
  priority = 1;
  name = "prob1";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Inequality);


  /* Setup second problem:
   *  -3*x1 + x2 = 2
   */
  A = Eigen::MatrixXd(1,solutionSpaceDimension);
  b = Eigen::VectorXd(1);
  D = Eigen::MatrixXd();
  d = Eigen::VectorXd();
  A << -3, 1;
  b << 2;
  priority = 2;
  name = "prob2";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Equality);

  /* Setup third problem:
   *  -4*x1 + x2 = -1
   */
  A = Eigen::MatrixXd(1,solutionSpaceDimension);
  b = Eigen::VectorXd(1);
  D = Eigen::MatrixXd();
  d = Eigen::VectorXd();
  A << -4, 1;
  b << -1;
  priority = 3;
  name = "prob3";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Equality);

  /* Setup fourth problem:
   *  min ||x||^2
   */
  A = Eigen::MatrixXd::Identity(solutionSpaceDimension, solutionSpaceDimension);
  b = Eigen::VectorXd::Zero(solutionSpaceDimension);
  D = Eigen::MatrixXd();
  d = Eigen::VectorXd();
  priority = 4;
  name = "minimize_x";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Equality);

  Eigen::VectorXd expectedSolution(solutionSpaceDimension);
  expectedSolution << 3.0, 11.0;

  Eigen::VectorXd computedSolution = Eigen::VectorXd::Zero(solutionSpaceDimension);
  solver.solveOptimization(computedSolution);

  NUMOPT_ASSERT_DOUBLE_MX_EQ(expectedSolution, computedSolution, 1e-3, unitTestName);
}

TEST(HierarchicalOptimizationQpNullSpaceProjectionTest, borderSolution) {

  std::string unitTestName{"constrainedProblem"};

  constexpr unsigned int solutionSpaceDimension = 2;
  std::unique_ptr<numopt_quadprog::ActiveSetFunctionMinimizer> minimizer(new numopt_quadprog::ActiveSetFunctionMinimizer());
  hopt::HierarchicalOptimizationQpNullSpaceProjection solver(solutionSpaceDimension, std::move(minimizer), true);

  Eigen::MatrixXd A, D;
  Eigen::VectorXd b, d;
  unsigned int priority;
  std::string name;

  constexpr double maxState = 10.0;

  /* Setup first problem:
   *  0 <= x1 <= a
   *  0 <= x2 <= a
   */
  A = Eigen::MatrixXd();
  b = Eigen::VectorXd();
  D.setZero(4,solutionSpaceDimension);
  d.setZero(4);

  D.topRows(2) = Eigen::Matrix2d::Identity();
  D.bottomRows(2) = -Eigen::Matrix2d::Identity();
  d.segment<solutionSpaceDimension>(0) = maxState * Eigen::Vector2d::Ones();
  d.segment<solutionSpaceDimension>(2) = Eigen::Vector2d::Zero();

  priority = 1;
  name = "prob1";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Inequality);


  /* Setup second problem:
   *  x1 + x2 = 2*maxState
   */
  A = Eigen::MatrixXd(1,solutionSpaceDimension);
  b = Eigen::VectorXd(1);
  D = Eigen::MatrixXd();
  d = Eigen::VectorXd();
  A << 1, 1;
  b << 2*maxState;
  priority = 2;
  name = "prob2";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Equality);

  Eigen::VectorXd expectedSolution(solutionSpaceDimension);
  expectedSolution << maxState, maxState;

  Eigen::VectorXd computedSolution = Eigen::VectorXd::Zero(solutionSpaceDimension);
  solver.solveOptimization(computedSolution);

  NUMOPT_ASSERT_DOUBLE_MX_EQ(expectedSolution, computedSolution, 1e-3, unitTestName);
}

TEST(HierarchicalOptimizationQpNullSpaceProjectionTest, borderSolutionMinimized) {

  std::string unitTestName{"constrainedProblem"};

  constexpr unsigned int solutionSpaceDimension = 2;
  std::unique_ptr<numopt_quadprog::ActiveSetFunctionMinimizer> minimizer(new numopt_quadprog::ActiveSetFunctionMinimizer());
  hopt::HierarchicalOptimizationQpNullSpaceProjection solver(solutionSpaceDimension, std::move(minimizer), true);

  Eigen::MatrixXd A, D;
  Eigen::VectorXd b, d;
  unsigned int priority;
  std::string name;

  constexpr double maxState = 10.0;

  /* Setup first problem:
   *  0 <= x1 <= a
   *  0 <= x2 <= a
   */
  A = Eigen::MatrixXd();
  b = Eigen::VectorXd();
  D.setZero(4,solutionSpaceDimension);
  d.setZero(4);

  D.topRows(2) = Eigen::Matrix2d::Identity();
  D.bottomRows(2) = -Eigen::Matrix2d::Identity();
  d.segment<solutionSpaceDimension>(0) = maxState * Eigen::Vector2d::Ones();
  d.segment<solutionSpaceDimension>(2) = Eigen::Vector2d::Zero();

  priority = 1;
  name = "prob1";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Inequality);


  /* Setup second problem:
   *  x1 + x2 = 2*maxState
   */
  A = Eigen::MatrixXd(1,solutionSpaceDimension);
  b = Eigen::VectorXd(1);
  D = Eigen::MatrixXd();
  d = Eigen::VectorXd();
  A << 1, 1;
  b << 2*maxState;
  priority = 2;
  name = "prob2";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Equality);

  /* Setup thrid problem:
   *  min ||x||^2
   */
  A = Eigen::MatrixXd::Identity(solutionSpaceDimension, solutionSpaceDimension);
  b = Eigen::VectorXd::Zero(solutionSpaceDimension);
  D = Eigen::MatrixXd();
  d = Eigen::VectorXd();
  priority = 4;
  name = "minimize_x";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Equality);

  Eigen::VectorXd expectedSolution(solutionSpaceDimension);
  expectedSolution << maxState, maxState;

  Eigen::VectorXd computedSolution = Eigen::VectorXd::Zero(solutionSpaceDimension);
  solver.solveOptimization(computedSolution);

  NUMOPT_ASSERT_DOUBLE_MX_EQ(expectedSolution, computedSolution, 1e-3, unitTestName);
}

TEST(HierarchicalOptimizationQpNullSpaceProjectionTest, outsideBorderSolutionMinimized) {

  std::string unitTestName{"constrainedProblem"};

  constexpr unsigned int solutionSpaceDimension = 2;
  std::unique_ptr<numopt_quadprog::ActiveSetFunctionMinimizer> minimizer(new numopt_quadprog::ActiveSetFunctionMinimizer());
  hopt::HierarchicalOptimizationQpNullSpaceProjection solver(solutionSpaceDimension, std::move(minimizer), true);

  Eigen::MatrixXd A, D;
  Eigen::VectorXd b, d;
  unsigned int priority;
  std::string name;

  constexpr double maxState = 10.0;

  /* Setup first problem:
   *  0 <= x1 <= a
   *  0 <= x2 <= a
   */
  A = Eigen::MatrixXd();
  b = Eigen::VectorXd();
  D.setZero(4,solutionSpaceDimension);
  d.setZero(4);

  D.topRows(2) = Eigen::Matrix2d::Identity();
  D.bottomRows(2) = -Eigen::Matrix2d::Identity();
  d.segment<solutionSpaceDimension>(0) = maxState * Eigen::Vector2d::Ones();
  d.segment<solutionSpaceDimension>(2) = Eigen::Vector2d::Zero();

  priority = 1;
  name = "prob1";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Inequality);


  /* Setup second problem:
   *  x1 + x2 = 3*maxState
   */
  A = Eigen::MatrixXd(1,solutionSpaceDimension);
  b = Eigen::VectorXd(1);
  D = Eigen::MatrixXd();
  d = Eigen::VectorXd();
  A << 1, 1;
  b << 3*maxState;
  priority = 2;
  name = "prob2";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Equality);

  /* Setup thrid problem:
   *  min ||x||^2
   */
  A = Eigen::MatrixXd::Identity(solutionSpaceDimension, solutionSpaceDimension);
  b = Eigen::VectorXd::Zero(solutionSpaceDimension);
  D = Eigen::MatrixXd();
  d = Eigen::VectorXd();
  priority = 4;
  name = "minimize_x";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Equality);

  Eigen::VectorXd expectedSolution(solutionSpaceDimension);
  expectedSolution << maxState, maxState;

  Eigen::VectorXd computedSolution = Eigen::VectorXd::Zero(solutionSpaceDimension);
  solver.solveOptimization(computedSolution);

  NUMOPT_ASSERT_DOUBLE_MX_EQ(expectedSolution, computedSolution, 1e-3, unitTestName);
}

TEST(HierarchicalOptimizationQpNullSpaceProjectionTest, excavatorProblem) {

  std::string unitTestName{"excavatorProblem"};

  constexpr unsigned int solutionSpaceDimension = 4;
  std::unique_ptr<numopt_quadprog::ActiveSetFunctionMinimizer> minimizer(new numopt_quadprog::ActiveSetFunctionMinimizer());
  hopt::HierarchicalOptimizationQpNullSpaceProjection solver(solutionSpaceDimension, std::move(minimizer), true);

  Eigen::MatrixXd A, D;
  Eigen::VectorXd b, d;
  unsigned int priority;
  std::string name;

  constexpr double Fmin = 20.0;
  constexpr double Fmax = 1e6;

  /* Setup first problem:
   *  Fmin <= x_i <= Fmax, i = 1...4
   *  -> to max constraints
   *  x_i <= Fmax, i = 1...4
   *  -x_i <= -Fmin
   */
  A = Eigen::MatrixXd();
  b = Eigen::VectorXd();
  D.setZero(2*solutionSpaceDimension,solutionSpaceDimension);
  d.setZero(2*solutionSpaceDimension);

  D.topRows(solutionSpaceDimension) = Eigen::MatrixXd::Identity(solutionSpaceDimension, solutionSpaceDimension);
  D.bottomRows(solutionSpaceDimension) = -Eigen::MatrixXd::Identity(solutionSpaceDimension, solutionSpaceDimension);
  d.segment<solutionSpaceDimension>(0) = Fmax * Eigen::VectorXd::Ones(solutionSpaceDimension);
  d.segment<solutionSpaceDimension>(solutionSpaceDimension) = -Fmin * Eigen::VectorXd::Ones(solutionSpaceDimension);

  priority = 1;
  name = "limitForces";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Inequality);

  /* Setup second problem:
   *  sum x_i = F_des
   *  sum
   */
  constexpr unsigned int distributionDimension = 3;
  A = Eigen::MatrixXd(distributionDimension,solutionSpaceDimension);
  b = Eigen::VectorXd(distributionDimension);
  D = Eigen::MatrixXd();
  d = Eigen::VectorXd();
  A <<  1.0,  1.0, 1.0,  1.0,
        1.0, -1.0, 1.0, -1.0,
       -2.0, -2.0, 2.0,  2.0;
  b <<  0.0,
        2*Fmax,
        Fmax;
  priority = 2;
  name = "distribution";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Equality);

  /* Setup thrid problem:
   *  min ||x||^2
   */
  A = Eigen::MatrixXd::Identity(solutionSpaceDimension, solutionSpaceDimension);
  b = Eigen::VectorXd::Zero(solutionSpaceDimension);
  D = Eigen::MatrixXd();
  d = Eigen::VectorXd();
  priority = 3;
  name = "minimize_x";
  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Equality);

  Eigen::VectorXd expectedSolution(solutionSpaceDimension);
  expectedSolution << Fmax/4.0, Fmin, 3.0*Fmax/4.0, Fmin;

  Eigen::VectorXd computedSolution = Eigen::VectorXd::Zero(solutionSpaceDimension);
  solver.solveOptimization(computedSolution);
  NUMOPT_ASSERT_DOUBLE_MX_EQ(expectedSolution, computedSolution, 1e-3, unitTestName);
}



TEST(HierarchicalOptimizationQpNullSpaceProjectionTest, slackVariablesTest) {

  std::string unitTestName{"slackVariablesTest"};

  constexpr unsigned int solutionSpaceDimension = 2;
  std::unique_ptr<numopt_quadprog::ActiveSetFunctionMinimizer> minimizer(new numopt_quadprog::ActiveSetFunctionMinimizer());
  hopt::HierarchicalOptimizationQpNullSpaceProjection solver(solutionSpaceDimension, std::move(minimizer), true);

  Eigen::MatrixXd A, D;
  Eigen::VectorXd b, d;
  unsigned int priority;
  std::string name;

  /* Setup first problem:
   *  x_1 <= 5.0
   *  x_2 <= 5.0
   *  x_1 >= 1.0
   *  x_2 >= 1.0
   */
  const double maxX = 5.0;
  const double minX = 1.0;
  const unsigned int numIneqConstr = 4;

  A = Eigen::MatrixXd::Identity(solutionSpaceDimension, solutionSpaceDimension);
  b = Eigen::VectorXd::Zero(solutionSpaceDimension);
  D.setZero(numIneqConstr,solutionSpaceDimension);
  d.setZero(numIneqConstr);

  D.topRows(2) = Eigen::MatrixXd::Identity(solutionSpaceDimension, solutionSpaceDimension);
  D.bottomRows(2) = -Eigen::MatrixXd::Identity(solutionSpaceDimension, solutionSpaceDimension);
  d.topRows(2) = maxX * Eigen::VectorXd::Ones(2);
  d.bottomRows(2) = -minX * Eigen::VectorXd::Ones(2);

  priority = 1;
  name = "bounds";

  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Inequality);

  Eigen::VectorXd expectedSolution = Eigen::VectorXd::Zero(solutionSpaceDimension);
  expectedSolution.bottomRows(2) = Eigen::VectorXd::Ones(2);

  Eigen::VectorXd computedSolution = Eigen::VectorXd::Zero(solutionSpaceDimension);

  solver.solveOptimization(computedSolution);
  NUMOPT_ASSERT_DOUBLE_MX_EQ(expectedSolution, computedSolution, 1e-6, unitTestName);
}

TEST(HierarchicalOptimizationQpNullSpaceProjectionTest, illConditionedProblemTest) {
  /*
   * Having a null space for the first task will create a small eigenvalue, additionally having inequality constraints with slack variables
   * can result in high eigenvalues. The combination of the two creates a very badly conditioned optimization on the first task.
   * A second priority task is added to check that the solution remains valid when being passed to the next priority.
   */
  std::string unitTestName{"illConditionedProblemTest"};

  // Problem size
  constexpr unsigned int numParam = 2;
  constexpr unsigned int numIneqConstr = 1;

  // Full problem
  Eigen::MatrixXd A1(1, numParam), A2(1, numParam), D(numIneqConstr, numParam);
  Eigen::VectorXd b1(1), b2(1), d(numIneqConstr);

  // Tasks
  A1 << 1.0, 0.0;
  b1 << 0.0;
  A2 << 0.0, 1.0;
  b1 << 0.0;

  // Constraint
  D << 1.0, 0.0;
  d << -1.0;

  std::unique_ptr<numopt_quadprog::ActiveSetFunctionMinimizer> minimizer(new numopt_quadprog::ActiveSetFunctionMinimizer());
  hopt::HierarchicalOptimizationQpNullSpaceProjection solver(numParam, std::move(minimizer), true);
  solver.addOptimizationProblem("priority_1", 1, A1, b1, D, d, hopt::ConstraintType::Mixed);
  solver.addOptimizationProblem("priority_2", 2, A2, b2, Eigen::MatrixXd(), Eigen::VectorXd(), hopt::ConstraintType::Equality);

  Eigen::VectorXd expectedSolution(numParam);
  expectedSolution << -1.0, 0.0;

  Eigen::VectorXd computedSolution = Eigen::VectorXd::Zero(numParam);

  solver.solveOptimization(computedSolution);

  NUMOPT_ASSERT_DOUBLE_MX_EQ(expectedSolution, computedSolution, 1e-6, unitTestName);
}

TEST(HierarchicalOptimizationQpNullSpaceProjectionTest, largeAndDeepProblemTest) {
  /*
   * This test checks that splitting a single large quadratic problem into independent hierarchies still gives the same solution as solving
   * the problem in one go. Having a large amount of parameters and priorities reveals numerical issues that can arise when propagating
   * null-spaces and inequalities though the hierarchy of quadratic programs.
   *
   * To make sure that the full and hierarchical solution have the same optima, the first nunIneqConstr are independent of the rest of the
   * problem and will all be restricted by the inequality constraints. These equations are solved on the top priority.
   * The rest of the problem, solved either at lower priorities or at the same time, is unconstrained and therefore should have the same
   * solution for both approaches.
   */
  std::string unitTestName{"largeAndDeepProblemTest"};

  // Problem size
  constexpr unsigned int numberOfPriorities = 10;
  constexpr unsigned int numberOfEquationsPerPriority = 10;
  constexpr unsigned int numIneqConstr = numberOfEquationsPerPriority;
  constexpr unsigned int solutionSpaceDimension = numberOfPriorities * numberOfEquationsPerPriority;
  constexpr unsigned int numUnconstr = solutionSpaceDimension - numIneqConstr;

  // Full problem
  Eigen::MatrixXd A, D;
  Eigen::VectorXd b, d;

  srand(0); // Fix seed for reproducibility
  // Tasks
  A.setZero(solutionSpaceDimension, solutionSpaceDimension);
  A.topLeftCorner<numIneqConstr, numIneqConstr>().setRandom();
  A.bottomRightCorner<numUnconstr, numUnconstr>().setRandom();
  A.diagonal().array() += 10.0;   // Making A diagonally dominant ensures that tasks are linearly independent.
  b.setRandom(solutionSpaceDimension);
  b.array() += 1.0; // All b's are positive

  // Constraints -> based on the first numIneqConstr tasks to make sure that the constraints are active
  D = A.topRows<numIneqConstr>();
  d = b.head<numIneqConstr>() - Eigen::VectorXd::Ones(numIneqConstr);

  // Solve problem in a single priority
  std::unique_ptr<numopt_quadprog::ActiveSetFunctionMinimizer> fullProblemMinimizer(new numopt_quadprog::ActiveSetFunctionMinimizer());
  hopt::HierarchicalOptimizationQpNullSpaceProjection fullProblemSolver(solutionSpaceDimension, std::move(fullProblemMinimizer), true);
  fullProblemSolver.addOptimizationProblem("fullProblem", 1, A, b, D, d, hopt::ConstraintType::Mixed);

  Eigen::VectorXd computedFullSolution = Eigen::VectorXd::Zero(solutionSpaceDimension);
  fullProblemSolver.solveOptimization(computedFullSolution);

  // Solve problem as a hierarchy
  std::unique_ptr<numopt_quadprog::ActiveSetFunctionMinimizer> hierarchicalMinimizer(new numopt_quadprog::ActiveSetFunctionMinimizer());
  hopt::HierarchicalOptimizationQpNullSpaceProjection hierarchicalSolver(solutionSpaceDimension, std::move(hierarchicalMinimizer), true);

  // First priority is inequalities + first subset of equalities
  unsigned int  priority = 1;
  std::string name = "inequalities";
  hierarchicalSolver.addOptimizationProblem(name, priority, Eigen::MatrixXd(), Eigen::VectorXd(), D, d, hopt::ConstraintType::Inequality);

  // All tasks below are equality tasks by splitting the full A into several priorities.
  for (int i = 0; i<numberOfPriorities; i++) {
    name = "equality_" + std::to_string(i);
    const Eigen::MatrixXd A_subset = A.middleRows<numberOfEquationsPerPriority>(i*numberOfEquationsPerPriority);
    const Eigen::VectorXd b_subset = b.segment<numberOfEquationsPerPriority>(i*numberOfEquationsPerPriority);
    hierarchicalSolver.addOptimizationProblem(name, priority, A_subset, b_subset, Eigen::MatrixXd(), Eigen::VectorXd(), hopt::ConstraintType::Equality);
    priority++;
  }

  Eigen::VectorXd computedhierarchicalSolution = Eigen::VectorXd::Zero(solutionSpaceDimension);
  hierarchicalSolver.solveOptimization(computedhierarchicalSolution);

  NUMOPT_ASSERT_DOUBLE_MX_EQ(computedFullSolution, computedhierarchicalSolution, 1e-6, unitTestName);
}

