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
* @file    numopt_common_gtest.hpp
* @author  Christian Gehring
* @date    Aug 16, 2015
*/

#pragma once

#include <numopt_common/numopt_gtest_eigen.hpp>
#include <numopt_common/NonlinearObjectiveFunction.hpp>
#include <numopt_common/NonlinearFunctionConstraints.hpp>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <string>
#include <random>


namespace numopt_common {

static inline void test_objective_function_gradient(numopt_common::NonlinearObjectiveFunction* objective,
                                                    const std::string& testName,
                                                    numopt_common::Parameterization& params,
                                                    double tol = 1.0) {
  params.setRandom(params.getParams());
  numopt_common::Vector analyticalGradient;
  ASSERT_TRUE(objective->getLocalGradient(analyticalGradient, params));
  numopt_common::Vector estimatedGradient;
  ASSERT_TRUE(objective->estimateLocalGradient(estimatedGradient, params));
  EXPECT_EQ(estimatedGradient.size(), analyticalGradient.size());
  NUMOPT_ASSERT_DOUBLE_MX_EQ(estimatedGradient, analyticalGradient, tol, std::string{"Gradient of test "} + testName);
}

static inline void test_objective_function_hessian(numopt_common::NonlinearObjectiveFunction* objective,
                                                   const std::string& testName,
                                                   numopt_common::Parameterization& params,
                                                   double tol = 1.0,
                                                   double zeroThreshold = 1.0e-15) {
  params.setRandom(params.getParams());
  numopt_common::SparseMatrix analyticalHessian;
  ASSERT_TRUE(objective->getLocalHessian(analyticalHessian, params));
  numopt_common::SparseMatrix estimatedHessian;
  ASSERT_TRUE(objective->estimateLocalHessian(estimatedHessian, params));
  NUMOPT_ASSERT_DOUBLE_SPARSE_MX_EQ_ZT(estimatedHessian, analyticalHessian, tol, std::string{"Hessian of test "} + testName, zeroThreshold);
}

static inline void test_objective_function_lower_hessian(numopt_common::NonlinearObjectiveFunction* objective,
                                                         const std::string& testName,
                                                         numopt_common::Parameterization& params,
                                                         double tol = 1.0) {
  params.setRandom(params.getParams());
  numopt_common::SparseMatrix analyticalessian;
  ASSERT_TRUE(objective->getLocalHessian(analyticalessian, params));
  numopt_common::SparseMatrix estimatedHessian;
  ASSERT_TRUE(objective->estimateLocalHessian(estimatedHessian,params));
  numopt_common::SparseMatrix estimatedHessianLower = estimatedHessian.triangularView<Eigen::Lower>();
  numopt_common::SparseMatrix trueHessianLower = analyticalessian.triangularView<Eigen::Lower>();
  NUMOPT_ASSERT_DOUBLE_SPARSE_MX_EQ(estimatedHessianLower, trueHessianLower, tol, std::string{"Lower Hessian of test "} + testName);
}

static inline void test_objective_function_upper_hessian(numopt_common::NonlinearObjectiveFunction* objective,
                                                         const std::string& testName,
                                                         numopt_common::Parameterization& params,
                                                         double tol = 1.0) {
  params.setRandom(params.getParams());
  numopt_common::SparseMatrix analyticalessian;
  ASSERT_TRUE(objective->getLocalHessian(analyticalessian, params));
  numopt_common::SparseMatrix estimatedHessian;
  ASSERT_TRUE(objective->estimateLocalHessian(estimatedHessian,params));
  numopt_common::SparseMatrix estimatedHessianUpper = estimatedHessian.triangularView<Eigen::Upper>();
  numopt_common::SparseMatrix trueHessianUpper = analyticalessian.triangularView<Eigen::Upper>();
  NUMOPT_ASSERT_DOUBLE_SPARSE_MX_EQ(estimatedHessianUpper, trueHessianUpper, tol, std::string{"Upper Hessian of test "} + testName);
}

static inline void test_function_constraints_equality_jacobian(numopt_common::NonlinearFunctionConstraints* constraints,
                                                               const std::string& testName,
                                                               numopt_common::Parameterization& params,
                                                               double tol = 1.0) {
  params.setRandom(params.getParams());
  numopt_common::SparseMatrix analyticalJacobian;
  ASSERT_TRUE(constraints->getLocalEqualityConstraintJacobian(analyticalJacobian, params));
  numopt_common::SparseMatrix estimatedJacobian;
  ASSERT_TRUE(constraints->estimateLocalEqualityConstraintJacobian(estimatedJacobian,params));
  NUMOPT_ASSERT_DOUBLE_SPARSE_MX_EQ(estimatedJacobian, analyticalJacobian, tol, std::string{"Equality Constraints Jacobian of test "} + testName);
}

static inline void test_function_constraints_inequality_jacobian(numopt_common::NonlinearFunctionConstraints* constraints,
                                                                 const std::string& testName,
                                                                 numopt_common::Parameterization& params,
                                                                 double tol = 1.0) {
  params.setRandom(params.getParams());
  numopt_common::SparseMatrix analyticalJacobian;
  ASSERT_TRUE(constraints->getLocalInequalityConstraintJacobian(analyticalJacobian, params));
  numopt_common::SparseMatrix estimatedJacobian;
  ASSERT_TRUE(constraints->estimateLocalInequalityConstraintJacobian(estimatedJacobian, params));
  NUMOPT_ASSERT_DOUBLE_SPARSE_MX_EQ(estimatedJacobian, analyticalJacobian, tol, std::string{"Inequality Constraints Jacobian of test "} + testName);
}

static inline void test_function_constraints_equality_hessians(numopt_common::NonlinearFunctionConstraints* constraints,
                                                               const std::string& testName,
                                                               numopt_common::Parameterization& params,
                                                               double tol = 1.0) {
  params.setRandom(params.getParams());
  int nConstraints = constraints->getNumberOfEqualityConstraints();
  for (int i=0; i<nConstraints; i++) {
    numopt_common::SparseMatrix analyticalHessian;
    ASSERT_TRUE(constraints->getLocalEqualityConstraintHessian(analyticalHessian, params, i));
    numopt_common::SparseMatrix estimatedHessian;
    ASSERT_TRUE(constraints->estimateLocalEqualityConstraintHessian(estimatedHessian, params, i));
    NUMOPT_ASSERT_DOUBLE_SPARSE_MX_EQ(estimatedHessian, analyticalHessian, tol, std::to_string(i)+std::string{" equality constraint Hessian of test "} + testName);
  }
}

static inline void test_function_constraints_inequality_hessians(numopt_common::NonlinearFunctionConstraints* constraints,
                                                                 const std::string& testName,
                                                                 numopt_common::Parameterization& params,
                                                                 double tol = 1.0) {
  params.setRandom(params.getParams());
  int nConstraints = constraints->getNumberOfInequalityConstraints();
  for (int i=0; i<nConstraints; i++) {
    numopt_common::SparseMatrix analyticalHessian;
    ASSERT_TRUE(constraints->getLocalInequalityConstraintHessian(analyticalHessian, params, i));
    numopt_common::SparseMatrix estimatedHessian;
    ASSERT_TRUE(constraints->estimateLocalInequalityConstraintHessian(estimatedHessian, params, i));
    NUMOPT_ASSERT_DOUBLE_SPARSE_MX_EQ(estimatedHessian, analyticalHessian, tol, std::to_string(i)+std::string{" inequality constraint Hessian of test "} + testName);
  }
}

template < int Rows_>
static inline void setUniformRandom(Eigen::Matrix<double, Rows_, 1>& vector, double min, double max) {
  std::random_device  rand_dev;
  std::mt19937  generator(rand_dev());
  std::uniform_real_distribution<double>  distr(min, max);
  for (int i=0; i<vector.size(); i++) {
    vector(i) = distr(generator);
  }
}


} //namespace numopt_common
