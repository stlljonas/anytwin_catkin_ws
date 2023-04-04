/*!
* @file    HockSchittkowski47Problem_test.cpp
* @author  Anna-Maria Georgarakis, Christian Gehring
* @date    Oct 11, 2015
*/

#include <gtest/gtest.h>
#include <numopt_common/numopt_gtest.hpp>
#include <numopt_common/ParameterizationIdentity.hpp>

#include "numopt_problems/HockSchittkowski47Problem.hpp"

TEST(HockSchittkowski47Problem, objective_gradient_hessian)
{
  using namespace numopt_problems;
  using namespace numopt_common;
  HS47ObjectiveFunction objective;
  ParameterizationIdentity params(5);
  test_objective_function_gradient(&objective, "HockSchittkowski47ObjectiveFunction", params);
  test_objective_function_hessian(&objective, "HockSchittkowski47ObjectiveFunction", params);
}

TEST(HockSchittkowski47Problem, objective_start)
{
  using namespace numopt_problems;
  using namespace numopt_common;

  HS47ObjectiveFunction objective;

  ParameterizationIdentity params(5);
  auto& p = params.getParams();

  // recommended initial values
  p(0) = 2.0;
  p(1) = sqrt(2.0);
  p(2) = -1.0;
  p(3) = 2.0 - sqrt(2.0);
  p(4) = 0.5;

  double value = 99.0;
  ASSERT_TRUE(objective.computeValue(value, params));
  EXPECT_NEAR(20.7381, value, 1e-3);
}

TEST(HockSchittkowski47Problem, objective_solution)
{
  using namespace numopt_problems;
  using namespace numopt_common;

  HS47ObjectiveFunction objective;

  ParameterizationIdentity params(5);
  auto& p = params.getParams();

  // optimal solution
  p(0) = 1.0;
  p(1) = 1.0;
  p(2) = 1.0;
  p(3) = 1.0;
  p(4) = 1.0;

  double value = 99.0;
  ASSERT_TRUE(objective.computeValue(value, params));
  EXPECT_NEAR(0.0, value, 1e-3);
}

TEST(HockSchittkowski47Problem, constraints_jacobian)
{
  using namespace numopt_problems;
  using namespace numopt_common;
  ParameterizationIdentity params(5);

  HS47FunctionConstraints constraints;
  test_function_constraints_inequality_jacobian(&constraints, "HockSchittkowski47FunctionConstraints", params);
  test_function_constraints_equality_jacobian(&constraints, "HockSchittkowski47FunctionConstraints", params);
}

TEST(HockSchittkowski47Problem, equalityConstraints)
{
  using namespace numopt_problems;
  using namespace numopt_common;

  HS47FunctionConstraints constraints;

  ParameterizationIdentity params(5);
  auto& p = params.getParams();
  // optimal solution
  p(0) = 1.0;
  p(1) = 1.0;
  p(2) = 1.0;
  p(3) = 1.0;
  p(4) = 1.0;

  Vector values;
  ASSERT_TRUE(constraints.getEqualityConstraintValues(values, params));
  Eigen::VectorXd targetValues;
  ASSERT_TRUE(constraints.getEqualityConstraintTargetValues(targetValues));
  NUMOPT_ASSERT_DOUBLE_MX_EQ(targetValues, values, 1e-3, "HockSchittkowski47 equality constraints");
}

TEST(HockSchittkowski47Problem, constraints_hessians)
{
  using namespace numopt_problems;
  using namespace numopt_common;

  HS47FunctionConstraints constraints;
  ParameterizationIdentity params(5);
  test_function_constraints_inequality_hessians(&constraints, "HockSchittkowski47FunctionConstraints", params, 1.0);
  test_function_constraints_equality_hessians(&constraints, "HockSchittkowski47FunctionConstraints", params, 1.0);
}


