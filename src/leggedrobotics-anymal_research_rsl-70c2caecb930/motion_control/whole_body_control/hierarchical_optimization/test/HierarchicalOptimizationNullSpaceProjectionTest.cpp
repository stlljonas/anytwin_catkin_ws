/*!
* @file    HierarchicalOptimizationNullSpaceProjectionTest.cpp
* @author  Christian Gehring
* @date    Feb, 2016
*/

// Unit test facility
#include <gtest/gtest.h>

// numerical optimization
#include <numopt_common/numopt_gtest.hpp>

// hierarchical optimization
#include <hierarchical_optimization/HierarchicalOptimizationNullSpaceProjection.hpp>


TEST(HierarchicalOptimizationNullSpaceProjectionTest, oneEqProblem)
{
  using namespace hopt;

  std::string name{"oneEqProblem"};

  constexpr unsigned int solutionDimension = 3;

  HierarchicalOptimizationNullSpaceProjection solver(solutionDimension);

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3,solutionDimension);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(3);

  Eigen::MatrixXd D;
  Eigen::VectorXd d;

  unsigned int priority = 1;

  Eigen::VectorXd x = Eigen::VectorXd::Ones(solutionDimension);

  solver.addOptimizationProblem(name, priority, A, b, D, d, hopt::ConstraintType::Equality);
  ASSERT_TRUE(solver.solveOptimization(x));

  Eigen::VectorXd sol = Eigen::VectorXd::Zero(solutionDimension);
  NUMOPT_ASSERT_DOUBLE_MX_EQ(x, sol, 1e-3, name);
}
