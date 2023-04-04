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
* @file    SQPFunctionMinimizer.hpp
* @author  Stelian Coros, Christian Gehring, Peter Fankhauser
* @date    Aug 16, 2015
*/
#pragma once

// numopt common
#include <numopt_common/numopt_common.hpp>
#include <numopt_common/ConstrainedNonlinearProblem.hpp>
#include <numopt_common/ConstrainedNonlinearProblemSolver.hpp>
#include <numopt_common/QuadraticProblem.hpp>
#include <numopt_common/QuadraticProblemSolver.hpp>

// stl
#include <memory>
#include <functional>

namespace numopt_sqp {

/**
  Use the Sequential Quadratic Programming method to optimize a function, subject to constraints.

  Task: Find p that minimize f(p), such that Ap = b and d <= Cp <= f
  This means df/dp(p) = 0. SQP works similar as Newton iteration, i.e. approximates the environment of f as
  f(p + dp) ~ f(p) + df/dp(p)*dp + 1/2 dp' * d/dp(df/dp)(p) * dp
  Which is minimized when df/dp is closest to zero within the constraints.

  SQP hence solves a sequence of QP problems of the form
    min d/dp(df/dp)*dp + df/dp, s.t. A(p + dp) = b and C(p + dp) <= f
  which gives us the stepping direction dp within the constraint manifold. Iterating the above will
  hopefully yield p that minimizes f.

  A warning: Convergence can be improved in some cases by doing line search in the direction of dp.
  However, this can give us intermediate points outside the constraint manifold, which is bad.
  If you're unsure whether your constraints are non-convex, set 'maxLineSearchIterations' to 0.
*/
class SQPFunctionMinimizer: public numopt_common::ConstrainedNonlinearProblemSolver {
 public:
  /*!
   * @param maxIterations               maximum number of iterations
   * @param qpSolver                    Quadratic Program solver
   * @param solveResidual               abortion criterium
   * @param solveFunctionValue          abortion criterium (if function value is lower than this value)
   * @param printOutput                 print infomrations to terminal
   * @param checkConstraints            check satisfaction of equality and inequality constraints
   */
  explicit SQPFunctionMinimizer(
      std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver,
      int maxIterations = 2000,
      double solveResidual=0.0001,
      unsigned int maxLineSearchIterations = 5,
      double solveFunctionValue = -DBL_MAX,
      bool printOutput = false,
      bool checkConstraints = false);

  ~SQPFunctionMinimizer() override = default;

  /**
    min f(p) subject to the constraints...
  */
  bool minimize(numopt_common::ConstrainedNonlinearProblem* function,
                numopt_common::Parameterization& params,
                double &functionValue,
                const std::string& fileName = "",
                bool printProblemToFile = false) override;

  /*!
   * Registers a callback function that is called after every iteration step
   * of the minimizer. Use this method for debugging and visualizations or leave
   * it unregistered if not used.
   * @param callback the callback method to be registered.
   */
  void registerOptimizationStepCallback(std::function<void(const size_t,
                                        const numopt_common::Parameterization&,
                                        const double, const bool)> callback);

  /*!
   * Registers a callback function that is called at the beginning of every
   * iteration step of the minimizer.
   * @param callback the callback method to be registered.
   */
  void registerOptimizationStepInitCallback(std::function<void(const numopt_common::Parameterization&)> callback);

  void setSolveResidual(double solveResidual);
  void setPrintOutput(bool isPrinting);
  void setCheckConstraints(bool checkConstraints);
  void setMaxIterations(int maxIterations);

  void stop() override;

private:
  /**
   * We search for dp which solves the system of equations
   *   H*dp + grad == 0
   * in a least squares sense while maintaining the constraints A*p = b and d <= C*p <= f.
   * This means we want to solve
   *  min |H*dp + grad|^2,
   *   s.t. A*(p + dp) = b,
   *        d <= C*(p + dp) <= f
   *        l <= p + dp <= u
   */
  bool computeSearchDirection(numopt_common::ConstrainedNonlinearProblem* problem,
                              const numopt_common::SparseMatrix& hessian,
                              const numopt_common::Vector& gradient,
                              const numopt_common::Parameterization &p,
                              numopt_common::Vector &dp,
                              const numopt_common::SparseMatrix& A,
                              const numopt_common::Vector& b,
                              const numopt_common::Vector& d,
                              const numopt_common::SparseMatrix& C,
                              const numopt_common::Vector& f,
                              const numopt_common::Vector& l,
                              const numopt_common::Vector& u,
                              bool useParameterBounds,
                              unsigned int iter);

  /*! @returns alpha for computing next parameter set:  p(i+1) = p(i) + alpha*dp.
   *           This implementation of the line search works best when only a cost function
   *           is specified in the optimization problem.
   *
   * @param function  objective function
   * @param p         current parameter set
   * @param dp        parameter variation
   * @param maxSteps  maximum number of steps the line search should do in worst case.
   *
   */
  virtual bool doLineSearch(double& stepSize,
                            numopt_common::ConstrainedNonlinearProblem* const problem,
                            numopt_common::Parameterization& p,
                            const numopt_common::Delta& dp,
                            unsigned int sqp_iter);

protected:
  //! Print local QP matrices to file.
  bool printToFile(std::ofstream& file, unsigned int sqp_iter);

  // QP solver.
  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver_;

  // QP problem formulation.
  numopt_common::QuadraticProblem qpProblem_;

  // Callback called after completing an SQP step.
  std::function<void(const size_t, const numopt_common::Parameterization&, const double, bool)> optimizationStepCallback_;

  // Callback called before executing tan SQP step.
  std::function<void(const numopt_common::Parameterization&)> optimizationStepInitCallback_;

  //! Line Search options.
  unsigned int maxLineSearchIterations_;

  // SQP options
  int maxIterations_;
  double solveResidual_;
  double solveFunctionValue_;
  bool printOutput_;
  bool checkConstraints_;

  //! Flag for terminating the optimization.
  bool interrupt_;
};

} // namespace sooqp
