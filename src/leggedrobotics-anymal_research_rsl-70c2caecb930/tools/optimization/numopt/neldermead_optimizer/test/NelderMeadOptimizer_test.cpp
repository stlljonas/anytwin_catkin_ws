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
* @file    NelderMeadOptimizer_test.cpp
* @author  Dario Bellicoso
* @date    Jan 19, 2016
*/


#include "neldermead_optimizer/NelderMeadOptimizer.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/SparseCore>


namespace test_functions {

// A random intercept for the test functions.
static double intercept = (double)rand()/RAND_MAX;

// f(x) = x'*x + n
static double quadratic(const Eigen::VectorXd& x) {
  return (x.dot(x) + intercept);
}

// f(x) = -x'*x + n
static double quadratic_neg(const Eigen::VectorXd& x) {
  return (-(x.dot(x)) + intercept);
}

} /* namespace test_functions */

TEST(NelderMeadOptimizerTest, QuadraticFunctionMinimzer) {
  // Setup the optimizer.
  constexpr unsigned int problemDimension = 2;
  neldermead_optimizer::NelderMeadOptimizer optimizer;
  Eigen::VectorXd initialGuess = Eigen::VectorXd::Zero(problemDimension);
  Eigen::VectorXd xOpt(problemDimension);
  double fOpt;

  // Solve.
  optimizer.optimize(fOpt, xOpt, initialGuess, std::bind(&test_functions::quadratic, std::placeholders::_1), true);

  // Compare the computed solution with the expected one.
  EXPECT_NEAR(test_functions::intercept, fOpt, 1e-10);
}


TEST(NelderMeadOptimizerTest, QuadraticFunctionMaximizer) {
  // Setup the optimizer.
  constexpr unsigned int problemDimension = 2;
  neldermead_optimizer::NelderMeadOptimizer optimizer;
  Eigen::VectorXd initialGuess = Eigen::VectorXd::Zero(problemDimension);
  Eigen::VectorXd xOpt(problemDimension);
  double fOpt;

  // Solve.
  optimizer.optimize(fOpt, xOpt, initialGuess, std::bind(&test_functions::quadratic_neg, std::placeholders::_1), false);

  // Compare the computed solution with the expected one.
  EXPECT_NEAR(test_functions::intercept, fOpt, 1e-10);
}
