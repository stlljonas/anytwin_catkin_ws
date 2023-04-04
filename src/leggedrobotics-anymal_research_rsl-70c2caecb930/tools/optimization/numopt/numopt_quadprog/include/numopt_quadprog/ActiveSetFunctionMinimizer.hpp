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
* @file    ActiveSetFunctionMinimizer.hpp
* @author  Dario Bellicoso
* @date    Jan 8, 2016
*/
#pragma once

// numopt
#include "numopt_common/QuadraticProblemSolver.hpp"

// active set QP
#include <quadprog_catkin/ASQP.hpp>

namespace numopt_quadprog {

class ActiveSetFunctionMinimizer: public numopt_common::QuadraticProblemSolver {
 public:

  /*! ASQP
   * @param maxIter             termination after max iteration is exceeded
   * @param isHessianConstant   true if Hessian remains constant over all nonlinear iterations
   */
  ActiveSetFunctionMinimizer(int maxIter = 1000, bool isHessianConstant = false);
  ~ActiveSetFunctionMinimizer() override = default;

  bool minimize(numopt_common::QuadraticProblem* problem,
                numopt_common::Parameterization& p,
                double& functionValue,
                unsigned int nonlinIter = 0) override;

  void updateActiveConstraints();
  void useLastActiveConstraints(bool useSet);

  void printSolverState();

  void stop() override;

  bool isValid(const numopt_common::Vector& vec) const;
  bool isValid(const numopt_common::SparseMatrix& mat) const;
  bool isValid() const;

 private:
  //! The cost function evaluated at the optimum.
  double result_;

  //! The definition of infinity.
  double infinity_;

  //! The cost function hessian.
  numopt_common::SparseMatrix Q_;

  //! The cost function linear term,
  numopt_common::Vector c_;

  //! The equality constraints Jacobian.
  numopt_common::SparseMatrix A_;

  //! The inequality constraints Jacobian.
  numopt_common::SparseMatrix D_;

  //! The equality constraint target values.
  numopt_common::Vector b_;

  //! The inequality constraint minimum values.
  numopt_common::Vector d_;

  //! The inequality constraint maximum values.
  numopt_common::Vector f_;

  //! The lower bound values.
  numopt_common::Vector l_;

  //! The upper bound values.
  numopt_common::Vector u_;

  //! The optimization parameters.
  numopt_common::Params p_;

  //! The redefinition of the inequality constraints Jacobian such that Dhat*x <= fhat.
  numopt_common::SparseMatrix Dhat_;

  //! The redefinition of the inequality constraints max values such that Dhat*x <= fhat.
  Eigen::VectorXd fhat_;

  //! The active set minimizer.
  asqp::ASQP minimizer_;

  bool useSet_;

  asqp::ActiveSet activeSet_;

  //! Flag for terminating the optimization.
  bool interrupt_;

  //! If true, QP matrices are checked for nan/inf values as well as for consistent dimensions.
  bool checkQP_;
};

} /* namespace numopt_quadprog */

