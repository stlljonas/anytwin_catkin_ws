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
* @file    ConstrainedNonlinearProblem.hpp
* @author  Stelian Coros, Christian Gehring
* @date    Aug 16, 2015
*/
#pragma once

#include <numopt_common/NonlinearFunctionConstraints.hpp>
#include <numopt_common/NonlinearObjectiveFunction.hpp>
#include <numopt_common/Problem.hpp>

#include <memory>

namespace numopt_common {

//! Constrained Nonlinear Problem
/*!
 *  min f(x)
 *   x \in C
 */
class ConstrainedNonlinearProblem : public Problem
{
 public:
  ConstrainedNonlinearProblem(std::shared_ptr<NonlinearObjectiveFunction> objectiveFunction,
                              std::shared_ptr<NonlinearFunctionConstraints> functionConstraints);
  ~ConstrainedNonlinearProblem() override = default;

  NonlinearObjectiveFunction* getObjectiveFunctionPtr();
  NonlinearFunctionConstraints* getFunctionConstraintsPtr();

  virtual bool getLocalHessianOfLagrangian(SparseMatrix& hessian, const Parameterization& params) const;
  bool getGlobalHessianOfLagrangian(SparseMatrix& hessian, const Parameterization& params) const;

  /**
   * Calculate the lower part of the local Hessian of the Lagrangian function.
   * This function assumes that all Lagrange multipliers are equal to one.
   */
  bool getLowerLocalHessianOfLagrangian(SparseMatrix& hessian, const Parameterization& params) const;

  /**
   * Calculate the lower part of the local Hessian of the Lagrangian function.
   * The objective part of the Lagrangian is to be multiplied with the scalar
   * factor obj_factor and each of the constraint parts are to be multiplied with
   * their corresponding lagrange multiplier lambda(i).
   */
  virtual bool getLowerLocalHessianOfLagrangian(SparseMatrix& hessian, const Parameterization& params,
      Scalar obj_factor, const Vector& lambda) const;
  bool getLowerGlobalHessianOfLagrangian(SparseMatrix& hessian, const Parameterization& params) const;
  bool getLowerGlobalHessianOfLagrangian(SparseMatrix& hessian, const Parameterization& params,
      Scalar obj_factor, const Vector& lambda) const;




 protected:
  std::shared_ptr<NonlinearObjectiveFunction> objective_;
  std::shared_ptr<NonlinearFunctionConstraints> constraints_;
};

} /* namespace numopt */
