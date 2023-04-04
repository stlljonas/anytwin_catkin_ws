/*!
* @file    HockSchittkowski03Problem_test.cpp
* @author  Anna-Maria Georgarakis, Christian Gehring
* @date    Oct 11, 2015
*/

#include <gtest/gtest.h>
#include <numopt_common/numopt_gtest.hpp>
#include <numopt_common/ParameterizationIdentity.hpp>
#include "numopt_problems/HockSchittkowski03Problem.hpp"

TEST(HockSchittkowski03Problem, objective_start)
{
  using namespace numopt_problems;
  using namespace numopt_common;

  HS03ObjectiveFunction objective;

  // recommended initial values
  ParameterizationIdentity params(2);
  params.getParams()(0) = 10.0;
  params.getParams()(1) = 1.0;

  double value = 99.0;
  ASSERT_TRUE(objective.computeValue(value, params));
  EXPECT_NEAR(1.00081, value, 1e-3);
}

TEST(HockSchittkowski03Problem, objective_solution)
{
  using namespace numopt_problems;
  using namespace numopt_common;

  HS03ObjectiveFunction objective;

  // optimal solution
  ParameterizationIdentity params(2);
  params.getParams()(0) = 0.0;
  params.getParams()(1) = 0.0;

  double value = 99.0;
  ASSERT_TRUE(objective.computeValue(value, params));
  EXPECT_NEAR(0.0, value, 1e-3);
}

TEST(HockSchittkowski03Problem, objective_gradient_hessian)
{
  using namespace numopt_problems;
  using namespace numopt_common;

  HS03ObjectiveFunction objective;
  ParameterizationIdentity params(2);
  test_objective_function_gradient(&objective, "HockSchittkowski03ObjectiveFunction", params);
  test_objective_function_hessian(&objective, "HockSchittkowski03ObjectiveFunction", params);
}


TEST(HockSchittkowski03Problem, problem_objective_gradient_hessian)
{
  using namespace numopt_problems;
  using namespace numopt_common;

  HockSchittkowski03Problem problem(1.0e-3);
  ParameterizationIdentity params(2);
  test_objective_function_gradient(problem.getObjectiveFunctionPtr(), "HockSchittkowski03ObjectiveFunction", params);
  test_objective_function_hessian(problem.getObjectiveFunctionPtr(), "HockSchittkowski03ObjectiveFunction", params);
}

TEST(HockSchittkowski03Problem, constraints_jacobian)
{
  using namespace numopt_problems;
  using namespace numopt_common;

  HS03FunctionConstraints constraints;
  ParameterizationIdentity params(2);
  test_function_constraints_inequality_jacobian(&constraints, "HockSchittkowski03FunctionConstraints", params);
  test_function_constraints_equality_jacobian(&constraints, "HockSchittkowski03FunctionConstraints", params);
}

TEST(HockSchittkowski03Problem, constraints_hessians)
{
  using namespace numopt_problems;
  using namespace numopt_common;

  HS03FunctionConstraints constraints;
  ParameterizationIdentity params(2);
  test_function_constraints_inequality_hessians(&constraints, "HockSchittkowski03FunctionConstraints", params, 1.0);
  test_function_constraints_equality_hessians(&constraints, "HockSchittkowski03FunctionConstraints", params, 1.0);
}


