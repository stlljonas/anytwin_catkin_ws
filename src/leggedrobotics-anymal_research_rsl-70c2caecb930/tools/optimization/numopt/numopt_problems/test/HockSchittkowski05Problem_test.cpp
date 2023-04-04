/*!
* @file    HockSchittkowski05Problem_test.cpp
* @author  Anna-Maria Georgarakis, Christian Gehring
* @date    Oct 11, 2015
*/

#include <gtest/gtest.h>
#include <numopt_common/numopt_gtest.hpp>
#include <numopt_common/ParameterizationIdentity.hpp>
#include "numopt_problems/HockSchittkowski05Problem.hpp"

TEST(HockSchittkowski05Problem, objective_gradient_hessian)
{
  using namespace numopt_problems;
  using namespace numopt_common;

  HS05ObjectiveFunction objective;
  ParameterizationIdentity params(2);

  test_objective_function_gradient(&objective, "HockSchittkowski05ObjectiveFunction", params);
  test_objective_function_hessian(&objective, "HockSchittkowski05ObjectiveFunction", params);
}

TEST(HockSchittkowski05Problem, objective_start)
{
  using namespace numopt_problems;
  using namespace numopt_common;

  HS05ObjectiveFunction objective;
  ParameterizationIdentity params(2);
  auto& p = params.getParams();

  // recommended initial values
  p(0) = 0.0;
  p(1) = 0.0;

  double value = 99.0;
  ASSERT_TRUE(objective.computeValue(value, params));
  EXPECT_NEAR(1.0, value, 1e-3);
}

TEST(HockSchittkowski05Problem, objective_solution)
{
  using namespace numopt_problems;
  using namespace numopt_common;

  HS05ObjectiveFunction objective;
  ParameterizationIdentity params(2);
  auto& p = params.getParams();

  // optimal solution
  p(0) = -M_PI/3.0 + 0.5;
  p(1) = -M_PI/3.0 - 0.5;

  double value = 99.0;
  ASSERT_TRUE(objective.computeValue(value, params));
  EXPECT_NEAR(-0.5*sqrt(3.0)-M_PI/3.0, value, 1e-3);
}

TEST(HockSchittkowski05Problem, constraints_jacobian)
{
  using namespace numopt_problems;
  using namespace numopt_common;

  HS05FunctionConstraints constraints;
  ParameterizationIdentity params(2);

  test_function_constraints_inequality_jacobian(&constraints, "HockSchittkowski05FunctionConstraints", params);
  test_function_constraints_equality_jacobian(&constraints, "HockSchittkowski05FunctionConstraints", params);
}

TEST(HockSchittkowski05Problem, equalityConstraints)
{
  using namespace numopt_problems;
  using namespace numopt_common;

  HS05FunctionConstraints constraints;
  ParameterizationIdentity params(2);

  auto& p = params.getParams();

  // recommended initial values
  p(0) = 0.0;
  p(1) = 0.0;

  Vector values;
  ASSERT_TRUE(constraints.getEqualityConstraintValues(values, params));
  Vector targetValues;
  ASSERT_TRUE(constraints.getEqualityConstraintTargetValues(targetValues));
  NUMOPT_ASSERT_DOUBLE_MX_EQ(targetValues, values, 1e-3, "HockSchittkowski05 equality constraints");
}

TEST(HockSchittkowski05Problem, constraints_hessians)
{
  using namespace numopt_problems;
  using namespace numopt_common;

  HS05FunctionConstraints constraints;
  ParameterizationIdentity params(2);
  test_function_constraints_inequality_hessians(&constraints, "HockSchittkowski05FunctionConstraints", params, 1.0);
  test_function_constraints_equality_hessians(&constraints, "HockSchittkowski05FunctionConstraints", params, 1.0);
}


