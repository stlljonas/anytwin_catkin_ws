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
* @file    ActiveSetFunctionMinimizer.cpp
* @author  Dario Bellicoso
* @date    Jan 8, 2016
*/

// numopt
#include "numopt_quadprog/ActiveSetFunctionMinimizer.hpp"

// stl
#include <iostream>

// assert
#include <assert.h>

namespace numopt_quadprog {

ActiveSetFunctionMinimizer::ActiveSetFunctionMinimizer(int maxIter, bool isHessianConstant)
    : result_(0.0),
      infinity_(std::numeric_limits<double>::infinity()),
      Q_(), c_(), A_(), D_(), b_(),
      d_(), f_(), l_(), u_(), p_(),
      minimizer_(maxIter, isHessianConstant),
      useSet_(false),
      activeSet_(),
      interrupt_(false),
      checkQP_(false)
{

}

bool ActiveSetFunctionMinimizer::minimize(numopt_common::QuadraticProblem* problem,
                                          numopt_common::Parameterization& params,
                                          double& functionValue,
                                          unsigned int nonlinIter) {
  if (interrupt_) { return false; }
  if (!problem->getQuadraticObjectiveFunctionPtr()->getGlobalHessian(Q_, params)) {
    return false;
  }

  if (interrupt_) { return false; }
  if (!problem->getQuadraticObjectiveFunctionPtr()->getLinearTerm(c_)) {
    return false;
  }

  if (interrupt_) { return false; }
  if (!problem->getFunctionConstraintsPtr()->getGlobalEqualityConstraintJacobian(A_, params)) {
    return false;
  }

  if (interrupt_) { return false; }
  if (!problem->getFunctionConstraintsPtr()->getGlobalInequalityConstraintJacobian(D_, params)) {
    return false;
  }

  if (interrupt_) { return false; }
  if (!problem->getFunctionConstraintsPtr()->getEqualityConstraintTargetValues(b_)){
    return false;
  }

  if (interrupt_) { return false; }
  if (!problem->getFunctionConstraintsPtr()->getInequalityConstraintMinValues(d_)){
    return false;
  }

  if (interrupt_) { return false; }
  if (!problem->getFunctionConstraintsPtr()->getInequalityConstraintMaxValues(f_)){
    return false;
  }

  if (interrupt_) { return false; }
  if (!problem->getFunctionConstraintsPtr()->getGlobalBoundConstraintMinValues(l_)){
    return false;
  }

  if (interrupt_) { return false; }
  if (!problem->getFunctionConstraintsPtr()->getGlobalBoundConstraintMaxValues(u_)){
    return false;
  }

  if (checkQP_) {
    if (!isValid()) { return false; }
  } else { assert(isValid()); }

  const auto problemDimension = Q_.rows();

  Dhat_.resize(f_.rows()+d_.rows()+l_.rows()+u_.rows(), problemDimension);
  assert(Dhat_.rows() >= D_.rows());
  fhat_.resize(f_.rows()+d_.rows()+l_.rows()+u_.rows());

  if (f_.rows() != 0) {
    Dhat_.topRows(f_.rows()) = D_;
    fhat_.topRows(f_.rows()) = f_;
  }

  if (d_.rows() != 0) {
    Dhat_.middleRows(f_.rows(), d_.rows()) = (-D_).eval();
    fhat_.middleRows(f_.rows(), d_.rows()) = -d_;
  }

  if (u_.rows() != 0) {
    numopt_common::SparseMatrix identityU(u_.rows(), u_.rows());
    identityU.setIdentity();
    Dhat_.middleRows(f_.rows()+d_.rows(), u_.rows()) = identityU;
    fhat_.middleRows(f_.rows()+d_.rows(), u_.rows()) = u_;
  }

  if (l_.rows() != 0) {
    numopt_common::SparseMatrix negIdentityL(l_.rows(), l_.rows());
    negIdentityL.setIdentity();
    negIdentityL *= -1.0;
    Dhat_.bottomRows(l_.rows()) = negIdentityL;
    fhat_.bottomRows(l_.rows()) = -l_;
  }

  p_ = params.getParams();
  params.setIdentity(p_); // todo: required?

  try {
    if (useSet_) {
      result_ = minimizer_.minimize(Q_, c_, A_, b_, Dhat_, fhat_, p_, &activeSet_, false, nonlinIter);
    } else {
      result_ = minimizer_.minimize(Q_, c_, A_, b_, Dhat_, fhat_, p_, nullptr, false, nonlinIter);
    }
  } catch (const std::runtime_error& error) {
    std::cout << "[ActiveSetFunctionMinimizer::minimize] Caught exception when solving the QP problem." << std::endl
              << "The exception was: " << error.what() << std::endl
              << "The solution will be reset to zero." << std::endl;
    // As a safety feature, set the result to zero.
    params.setIdentity(p_);
    result_ = infinity_;
  }
  params.getParams() = p_;
  problem->setCurrentBestSolution(params);
  functionValue = result_;

  return (result_ != infinity_);
}

void ActiveSetFunctionMinimizer::updateActiveConstraints() {
  activeSet_.A.setZero(Dhat_.rows());
  for (unsigned int k=0; k<Dhat_.rows(); k++) {
    const bool diff = std::abs(Dhat_.row(k).dot(p_) - fhat_(k)) < std::numeric_limits<double>::min();
    if (diff) {
      activeSet_.A(k) = 1;
    }
  }
  activeSet_.xStar = p_;
}

void ActiveSetFunctionMinimizer::useLastActiveConstraints(bool useSet) {
  useSet_ = useSet;
}

void ActiveSetFunctionMinimizer::printSolverState() {
  minimizer_.printState();
}

void ActiveSetFunctionMinimizer::stop() {
  interrupt_ = true;
  minimizer_.stop();
}

bool ActiveSetFunctionMinimizer::isValid(const numopt_common::Vector& vec) const {
  if (!vec.allFinite()) { return false; }
  return true;
}

bool ActiveSetFunctionMinimizer::isValid(const numopt_common::SparseMatrix& mat) const {
  if (!mat.toDense().allFinite()) { return false; }
  return true;
}

bool ActiveSetFunctionMinimizer::isValid() const {
  bool success = true;

  if (!isValid(Q_)) {
    std::cout << "[ActiveSetFunctionMinimizer::isValid] Hessian appears to be invalid.\n";
    success = false;
  }

  if (!isValid(c_)) {
    std::cout << "[ActiveSetFunctionMinimizer::isValid] Linear term appears to be invalid.\n";
    success = false;
  }

  if (!isValid(A_)) {
    std::cout << "[ActiveSetFunctionMinimizer::isValid] Equality constraints Jacobian appears to be invalid.\n";
    success = false;
  }

  if (!isValid(D_)) {
    std::cout << "[ActiveSetFunctionMinimizer::isValid] Inequality constraints Jacobian appears to be invalid.\n";
    success = false;
  }

  if (!isValid(b_)) {
    std::cout << "[ActiveSetFunctionMinimizer::isValid] Equality constraint target values appears to be invalid.\n";
    return false;
  }

  if (!isValid(d_)) {
    std::cout << "[ActiveSetFunctionMinimizer::isValid] Inequality constraint min values constraint target values appears to be invalid.\n";
    return false;
  }

  if (!isValid(f_)) {
    std::cout << "[ActiveSetFunctionMinimizer::isValid] Inequality constraint max values appears to be invalid.\n";
    success = false;
  }

  if (!isValid(l_)) {
    std::cout << "[ActiveSetFunctionMinimizer::isValid] Lower bound values appears to be invalid.\n";
    success = false;
  }

  if (!isValid(u_)) {
    std::cout << "[ActiveSetFunctionMinimizer::isValid] Upper bound values appears to be invalid.\n";
    return false;
  }

  // Check consistency of hessian and linear term.
  if(Q_.rows() != Q_.cols()) {
    std::cout << "[ActiveSetFunctionMinimizer::isValid] Wrong dimensions\n";
    success = false;
  };

  if(Q_.cols() != c_.rows()) {
    std::cout << "[ActiveSetFunctionMinimizer::isValid] Wrong dimensions\n";
    success = false;
  }

  // Check consistency of equality constraints.
  if(A_.rows() != b_.rows()) {
    std::cout << "[ActiveSetFunctionMinimizer::isValid] Wrong dimensions\n";
    success = false;
  }

  if(!((A_.cols() == 0) ||  (A_.cols() == Q_.cols()))) {
    std::cout << "[ActiveSetFunctionMinimizer::isValid] Wrong dimensions\n";
    success = false;
  }

  // Check consistency of inequality constraints.
  if(!( (D_.rows() == f_.rows()) || (f_.rows() == 0))) {
    std::cout << "[ActiveSetFunctionMinimizer::isValid] Wrong dimensions\n";
    success = false;
  }
  if(!( (D_.rows() == d_.rows()) || (d_.rows() == 0))) {
    std::cout << "[ActiveSetFunctionMinimizer::isValid] Wrong dimensions\n";
    success = false;
  }
  if(!( (D_.cols() == 0) ||  (D_.cols() == Q_.cols()))) {
    std::cout << "[ActiveSetFunctionMinimizer::isValid] Wrong dimensions\n";
    success = false;
  }

  // Check consistency of upper and lower bounds.
  if(!(u_.rows() == l_.rows())) {
    std::cout << "[ActiveSetFunctionMinimizer::isValid] Wrong dimensions\n";
    success = false;
  }

  if(!( (l_.rows() == 0) || (l_.rows() == Q_.cols()) )) {
    std::cout << "[ActiveSetFunctionMinimizer::isValid] Wrong dimensions\n";
    success = false;
  }

  if(!( (u_.rows() == 0) || (u_.rows() == Q_.cols()) )) {
    std::cout << "[ActiveSetFunctionMinimizer::isValid] Wrong dimensions\n";
    success = false;
  }

  return success;
}

} /* namespace numopt_quadprog */
