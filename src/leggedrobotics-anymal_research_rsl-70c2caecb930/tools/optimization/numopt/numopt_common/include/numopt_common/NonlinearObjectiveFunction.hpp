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
* @file    ObjectiveFunction.hpp
* @author  Stelian Coros, Christian Gehring
* @date    Aug 16, 2015
*/
#pragma once

#include "numopt_common/numopt_common.hpp"
#include "numopt_common/ObjectiveFunction.hpp"

namespace numopt_common {

/*! An interface class for a twice differentiable, non-linear objective function.
 * Derive this class and implement the function computeValue().
 * This class provides the gradient and Hessian of the objective function through finite-difference method by default.
 */
class NonlinearObjectiveFunction : public ObjectiveFunction {
public:
	NonlinearObjectiveFunction();
	virtual ~NonlinearObjectiveFunction();

	/*! Implement the objective function here.
	 * this should always return the current value of the objective function
	 */
	/*! This method computes the objective value
	 *  Implement the objective function.
	 *
	 * @param value       function value
	 * @param p           parameters
	 * @param newParams   true if this class has already seen the parameters
	 * @return  true if successful
	 */
	virtual bool computeValue(double& value, const Parameterization& p, bool newParams = true) = 0;

  /*! Computes the local gradient of the objective function.
   * Implement the gradient.
   *
   * @param gradient    gradient (column vector)
   * @param p           parameters
   * @param newParams   true if this class has already seen the parameters
   * @returns true if successful
   */
  virtual bool getLocalGradient(Vector& gradient, const Parameterization& p, bool newParams = true);

  /*! Computes the global gradient of the objective function.
   * By default, the method transforms the global gradient.
   *
   * @param gradient    gradient (column vector)
   * @param p           parameters
   * @param newParams   true if this class has already seen the parameters
   * @returns true if successful
   */
  virtual bool getGlobalGradient(Vector& gradient, const Parameterization& p, bool newParams = true);

	/*! Computes the local nxn Hessian matrix of the objective function.
	 * Implement the Hessian.
	 * Note that this function does not need to update the list of triplets.
	 *
	 * @param hessian     Hessian matrix
	 * @param p           parameters
	 * @param newParams   true if this class has already seen the parameters
	 * @returns true if successful
	 */
	virtual bool getLocalHessian(SparseMatrix& hessian, const Parameterization& p, bool newParams = true);

  /*! Computes the global nxn Hessian matrix of the objective function.
   * By default, this transforms the local Hessian.
   *
   * @param hessian     Hessian matrix
   * @param p           parameters
   * @param newParams   true if this class has already seen the parameters
   * @returns true if successful
   */
	virtual bool getGlobalHessian(SparseMatrix& hessian, const Parameterization& p, bool newParams = true);

  /*! Implement the Hessian here.
   * Note that this function does not need to update the Hessian matrix hessian_.
   * @param p   parameters
   * @returns a list of triplets that represent the Hessian
   * @see getHessianAt
   */
  virtual bool getLocalHessianTriplets(SMTriplets& triplets, const Parameterization& p, bool newParams = true);


	//! Estimates the gradient through finite-difference
	bool estimateLocalGradient(Vector& gradient, const Parameterization& p, double delta = 1.0e-8);

	//! Estimates the hessian through finite-difference
	bool estimateLocalHessian(SparseMatrix& hessian, const Parameterization& p, double delta = 1.0e-8);

  //! Estimates the hessian through finite-difference
	bool estimateLocalHessianTriplets(SMTriplets& hessianTriplets, const Parameterization& p, double delta = 1.0e-8);


};



} // namespace
