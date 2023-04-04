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
* @file    NonlinearFunctionConstraints.hpp
* @author  Stelian Coros, Christian Gehring
* @date    Aug 16, 2015
*/

#pragma once

#include "numopt_common/numopt_common.hpp"
#include "numopt_common/FunctionConstraints.hpp"
#include "numopt_common/Parameterization.hpp"

namespace numopt_common {

/*!
	A multi-dimensional non-linear function that expresses equality and inequality constraints
	that may be applied to an objective function.

	Equality constraints:    a(p) = b
	Inequality constraints:  d <= c(p) <= f
	Bound constraints:       l <= p <= u

*/
class NonlinearFunctionConstraints : public FunctionConstraints {
public:
	NonlinearFunctionConstraints();
	virtual ~NonlinearFunctionConstraints();

	/*
	 *  Equality constraints
	 */

	/*! @returns number of equality constraints
	 */
  virtual int getNumberOfEqualityConstraints();
  virtual void setNumberOfEqualityConstraints(int nEqualityConstraints);

  /*! Derive this method and compute the values of the equality constraints a(p)
   *
   * @param values         a(p) of the constraint a(p) = b
   * @param p              parameters
   * @param newParams      true if this class has already seen the parameters
   * @returns true if successful
   */
  virtual bool getEqualityConstraintValues(Vector& values, const Parameterization& p, bool newParams = true);


  /*! Derive this method and return the target values b of the equality constraint.
   *
   * @param values         b of the constraint a(p) = b
   * @param p              parameters
   * @param newParams      true if this class has already seen the parameters
   * @returns true if successful
   */
  virtual bool getEqualityConstraintTargetValues(Vector& values);

  /*! Derive this method and implement the local analytic Jacobian of the equality constraints,
   * otherwise it is estimated through finite-difference.
   *
   * @param jacobian        da/dp of the constraint a(p) = b
   * @param p               parameters
   * @param newParams       true if this class has already seen the parameters
   * @returns true if successful
   */
	virtual bool getLocalEqualityConstraintJacobian(SparseMatrix& jacobian, const Parameterization& p, bool newParams = true);

	/*! Computes the global analytic Jacobian of the equality constraints.
   * By default, it returns the tranformed local Jacobian.
   *
   * @param jacobian        da/dp of the constraint a(p) = b
   * @param p               parameters
   * @param newParams       true if this class has already seen the parameters
   * @returns true if successful
   */
	virtual bool getGlobalEqualityConstraintJacobian(SparseMatrix& jacobian, const Parameterization& p, bool newParams = true);


  virtual bool getLocalEqualityConstraintHessian(SparseMatrix& hessian, const Parameterization& p, int iConstraint, bool newParams = true);
  virtual bool getGlobalEqualityConstraintHessian(SparseMatrix& hessian, const Parameterization& p, int iConstraint, bool newParams = true);


  /*
   *  Inequality constraints
   */

  /*! @returns number of inequality constraints
   */
  virtual int getNumberOfInequalityConstraints();
  virtual void setNumberOfInequalityConstraints(int nInequalityConstraints);

  /*! Derive this method and compute ineqConstraintVals.
   * get the actual value of the equality constraints...
   * @returns c(p)  of d <= c(p) <= f
   */
  virtual bool getInequalityConstraintValues(Vector& values, const Parameterization& p, bool newParams = true);

  /*! @returns d of d <= c(p) <= f
   */
  virtual bool getInequalityConstraintMinValues(Vector& d);

  /*! @returns f of d <= c(p) <= f
   */
  virtual bool getInequalityConstraintMaxValues(Vector& f);


	/*! inequality constraints of the form d <= c(p) <= f
	 * @returns dc/dp
	 */
	virtual bool getLocalInequalityConstraintJacobian(SparseMatrix& jacobian, const Parameterization& p, bool newParams = true);
	virtual bool getGlobalInequalityConstraintJacobian(SparseMatrix& jacobian, const Parameterization& p, bool newParams = true);

  virtual bool getLocalInequalityConstraintHessian(SparseMatrix& hessian, const Parameterization& p, int iConstraint, bool newParams = true);
  virtual bool getGlobalInequalityConstraintHessian(SparseMatrix& hessian, const Parameterization& p, int iConstraint, bool newParams = true);

  /*
   *  Boundary constraints
   */

	/*! @returns min of constraint min <= dp <= max
	 */
	virtual bool getGlobalBoundConstraintMinValues(Vector& values);

  /*! @returns max of constraint min <= dp <= max
   */
	virtual bool getGlobalBoundConstraintMaxValues(Vector& values);


public:
  //!  Estimates the Jacobian of the equality constraints through finite-difference (FD)
	bool estimateLocalEqualityConstraintJacobian(SparseMatrix& J, const Parameterization& p);

  //!  Estimates the Hessian of an equality constraint through finite-difference (FD)
  bool estimateLocalEqualityConstraintHessian(SparseMatrix& H, const Parameterization& p, int iConstraint);

	//!  Estimates the Jacobian of the inequality constraints through finite-difference (FD)
  bool estimateLocalInequalityConstraintJacobian(SparseMatrix& J, const Parameterization& p);

  //!  Estimates the Hessian of an inequality constraint through finite-difference (FD)
  bool estimateLocalInequalityConstraintHessian(SparseMatrix& H, const Parameterization& p, int iConstraint);

  void checkDimensions(double nParameters);

  void printConstraintErrors(const Parameterization& p, double eqTol = 0.00001, double iqTol = 0.00001);

protected:
  //! Number of equality constraints
  int nEqualityConstraints_;
  //! Number of inequality constraints
  int nInequalityConstraints_;

};

} // namespace

