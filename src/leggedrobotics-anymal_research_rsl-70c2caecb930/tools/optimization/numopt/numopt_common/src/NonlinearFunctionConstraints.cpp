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
* @file    NonlinearFunctionConstraints.cpp
* @author  Stelian Coros, Christian Gehring
* @date    Aug 16, 2015
*/

#include "numopt_common/NonlinearFunctionConstraints.hpp"
#include "numopt_common/numopt_assert_macros.hpp"
#include <iostream>

namespace numopt_common {

NonlinearFunctionConstraints::NonlinearFunctionConstraints() :
    nEqualityConstraints_(0),
    nInequalityConstraints_(0)
{

}

NonlinearFunctionConstraints::~NonlinearFunctionConstraints(){

}

int NonlinearFunctionConstraints::getNumberOfEqualityConstraints() {
  return nEqualityConstraints_;
}

int NonlinearFunctionConstraints::getNumberOfInequalityConstraints() {
  return nInequalityConstraints_;
}

void NonlinearFunctionConstraints::setNumberOfEqualityConstraints(int nEqualityConstraints) {
  nEqualityConstraints_ = nEqualityConstraints;
}

void NonlinearFunctionConstraints::setNumberOfInequalityConstraints(int nInequalityConstraints) {
  nInequalityConstraints_ = nInequalityConstraints;
}

bool NonlinearFunctionConstraints::getEqualityConstraintValues(Vector& values, const Parameterization& p, bool newParams) {
  return (nEqualityConstraints_ == 0);
}

bool NonlinearFunctionConstraints::getEqualityConstraintTargetValues(Vector& values) {
  return (nEqualityConstraints_ == 0);
}

bool NonlinearFunctionConstraints::getLocalEqualityConstraintJacobian(numopt_common::SparseMatrix& eqConJacobian, const Parameterization& p, bool newParams) {
  if (nEqualityConstraints_ == 0) {
    eqConJacobian.resize(nEqualityConstraints_, p.getLocalSize());
    return true;
  }
  return estimateLocalEqualityConstraintJacobian(eqConJacobian, p);
}

bool NonlinearFunctionConstraints::getGlobalEqualityConstraintJacobian(numopt_common::SparseMatrix& jacobian, const Parameterization& p, bool newParams) {
  numopt_common::SparseMatrix localJacobian;
  if (!getLocalEqualityConstraintJacobian(localJacobian, p, newParams)) {
    return false;
  }
  numopt_common::SparseMatrix transformMatrix;
  if (!p.getTransformMatrixGlobalToLocal(transformMatrix, p.getParams())) {
    return false;
  }
  jacobian = localJacobian*transformMatrix;
  return true;
}

bool NonlinearFunctionConstraints::getLocalEqualityConstraintHessian(numopt_common::SparseMatrix& hessian,
                                                                     const Parameterization& p,
                                                                     int iConstraint,
                                                                     bool newParams) {
  if (nEqualityConstraints_ == 0) {
    hessian.resize(p.getLocalSize(), p.getLocalSize());
    return true;
  }
  return estimateLocalEqualityConstraintHessian(hessian, p, iConstraint);
}

bool NonlinearFunctionConstraints::getGlobalEqualityConstraintHessian(numopt_common::SparseMatrix& hessian, const Parameterization& p, int iConstraint, bool newParams) {
  numopt_common::SparseMatrix localHessian;
  if (!getLocalEqualityConstraintHessian(localHessian, p, iConstraint, newParams)) {
    return false;
  }
  numopt_common::SparseMatrix transformMatrix;
  if (!p.getTransformMatrixGlobalToLocal(transformMatrix, p.getParams())) {
    return false;
  }
  hessian = transformMatrix.transpose()*localHessian*transformMatrix;
  return true;
}

bool NonlinearFunctionConstraints::getLocalInequalityConstraintJacobian(numopt_common::SparseMatrix& ineqConJacobian, const Parameterization& p, bool newParams) {
  if (nInequalityConstraints_ == 0) {
    ineqConJacobian.resize(nInequalityConstraints_, p.getLocalSize());
    return true;
  }
  return estimateLocalInequalityConstraintJacobian(ineqConJacobian, p);
}

bool NonlinearFunctionConstraints::getGlobalInequalityConstraintJacobian(numopt_common::SparseMatrix& jacobian, const Parameterization& p, bool newParams) {
  numopt_common::SparseMatrix localJacobian;
  if (!getLocalInequalityConstraintJacobian(localJacobian, p, newParams)) {
    return false;
  }
  numopt_common::SparseMatrix transformMatrix;
  if (!p.getTransformMatrixGlobalToLocal(transformMatrix, p.getParams())) {
    return false;
  }
  jacobian = localJacobian*transformMatrix;
  return true;
}

bool NonlinearFunctionConstraints::getLocalInequalityConstraintHessian(numopt_common::SparseMatrix& hessian, const Parameterization& p, int iConstraint, bool newParams) {
  if (nInequalityConstraints_ == 0) {
    hessian.resize(p.getLocalSize(), p.getLocalSize());
    return true;
  }
  return estimateLocalInequalityConstraintHessian(hessian, p, iConstraint);
}

bool NonlinearFunctionConstraints::getGlobalInequalityConstraintHessian(numopt_common::SparseMatrix& hessian, const Parameterization& p, int iConstraint, bool newParams) {
  numopt_common::SparseMatrix localHessian;
  if (!getLocalInequalityConstraintHessian(localHessian, p, iConstraint, newParams)) {
    return false;
  }
  numopt_common::SparseMatrix transformMatrix;
  if (!p.getTransformMatrixGlobalToLocal(transformMatrix, p.getParams())) {
    return false;
  }
  hessian = transformMatrix.transpose()*localHessian*transformMatrix;
  return true;
}

bool NonlinearFunctionConstraints::getInequalityConstraintMinValues(Vector& values) {
  return (nInequalityConstraints_ == 0);
}

bool NonlinearFunctionConstraints::getInequalityConstraintMaxValues(Vector& values) {
  return (nInequalityConstraints_ == 0);
}

bool NonlinearFunctionConstraints::getInequalityConstraintValues(Vector& values, const Parameterization& p, bool newParams) {
  return (nInequalityConstraints_ == 0);
}

bool NonlinearFunctionConstraints::getGlobalBoundConstraintMinValues(Vector& values) {
  return false;
}

bool NonlinearFunctionConstraints::getGlobalBoundConstraintMaxValues(Vector& values) {
  return false;
}

bool NonlinearFunctionConstraints::estimateLocalEqualityConstraintJacobian(numopt_common::SparseMatrix& jacobian, const Parameterization& params){
  //this is a very slow method that evaluates the Jacobian of the objective function through FD...

	const int nConstraints = getNumberOfEqualityConstraints();
  const int nLocalParams = params.getLocalSize();
  jacobian.resize(nConstraints, nLocalParams);

	if (nConstraints > 0) {
    const double delta = 10e-6;
    const double one_over_two_delta = 1.0 / (2.0 * delta);

	  Parameterization* pSet = params.clone();
    std::vector<SMTriplet> tripletList;
    Vector C_P(nConstraints), C_M(nConstraints), J_i_col(nConstraints);

    // delta vector
    Vector dp = Vector::Zero(nLocalParams);

    for (int i=0; i<nLocalParams; i++) {
      dp(i) = delta;
      pSet->plus(pSet->getParams(), params.getParams(), dp);

      if(!getEqualityConstraintValues(C_P, *pSet)) {
        delete pSet;
        return false;
      }

      dp(i) = -delta;
      pSet->plus(pSet->getParams(), params.getParams(), dp);

      if(!getEqualityConstraintValues(C_M, *pSet)) {
        delete pSet;
        return false;
      }
      J_i_col = (C_P - C_M) * one_over_two_delta;

      // reset delta
      dp(i) = 0.0;

      //each vector is a column vector of the hessian, so copy it in place...
      for (int j=0;j<nConstraints;j++) {
        if (!IS_ZERO(J_i_col(j))) {
          tripletList.push_back(SMTriplet(j, i, J_i_col(j)));
        }
      }
    }
    jacobian.setFromTriplets(tripletList.begin(), tripletList.end());
    delete pSet;
	}
	return true;
}


bool NonlinearFunctionConstraints::estimateLocalEqualityConstraintHessian(numopt_common::SparseMatrix& hessian,
                                                                     const Parameterization& params,
                                                                     int iConstraint) {

  // this is a very slow method that evaluates the hessian of the objective function through FD...
  SMTriplets hessianTriplets;
  numopt_common::SparseMatrix jacobian;
  Parameterization* pSet = params.clone();

  double delta = 10e-6;
  const double one_over_two_delta = 1.0 / (2.0 * delta);
  const int nLocalParams = params.getLocalSize();

  Vector C_P(nLocalParams), C_M(nLocalParams), H_i_col(nLocalParams);

  Vector dp = Vector::Zero(nLocalParams);

  for (int i=0;i<nLocalParams;i++) {
    dp(i) = delta;
    pSet->plus(pSet->getParams(), params.getParams(), dp);
    if (!getLocalEqualityConstraintJacobian(jacobian, *pSet)) {
      delete pSet;
      return false;
    }
    C_P = jacobian.toDense().row(iConstraint).transpose();


    dp(i) = -delta;
    pSet->plus(pSet->getParams(), params.getParams(), dp);
    if (!getLocalEqualityConstraintJacobian(jacobian, *pSet)) {
      delete pSet;
      return false;
    }
    C_M = jacobian.toDense().row(iConstraint).transpose();


    // reset delta
    dp(i) = 0.0;

    //and compute the row of the hessian
    H_i_col = (C_P - C_M) * one_over_two_delta;


    //each vector is a column vector of the hessian, so copy it in place...
    for (int j = 0;j < nLocalParams;j++) {
      if (!IS_ZERO(H_i_col(j))) {
        hessianTriplets.push_back(SMTriplet(j, i, H_i_col(j)));
      }
    }
  }
  delete pSet;
  hessian.resize(nLocalParams, nLocalParams);
  hessian.setFromTriplets(hessianTriplets.begin(), hessianTriplets.end());
  return true;
}


bool NonlinearFunctionConstraints::estimateLocalInequalityConstraintJacobian(SparseMatrix& jacobian, const Parameterization& params){
  //this is a very slow method that evaluates the Jacobian of the objective function through FD...

   const int nConstraints = getNumberOfInequalityConstraints();
   const int nLocalParams = params.getLocalSize();
   jacobian.resize(nConstraints, nLocalParams);
//   std::cout << "estimateLocalInequalityConstraintJacobian: nCosntraints: " << nConstraints << std::endl;
//   std::cout << "estimateLocalInequalityConstraintJacobian: nLocalParams: " << nLocalParams << std::endl;
   if (nConstraints > 0) {
     const double delta = 10e-6;
     const double one_over_two_delta = 1.0 / (2.0 * delta);

     Parameterization* pSet = params.clone();
     std::vector<SMTriplet> tripletList;
     Vector C_P(nConstraints), C_M(nConstraints), J_i_col(nConstraints);

     // delta vector
     Vector dp = Vector::Zero(nLocalParams);

     for (int i=0; i<nLocalParams; i++) {
       dp(i) = delta;
       pSet->plus(pSet->getParams(), params.getParams(), dp);

       if(!getInequalityConstraintValues(C_P, *pSet)) {
         delete pSet;
         return false;
       }

       dp(i) = -delta;
       pSet->plus(pSet->getParams(), params.getParams(), dp);

       if(!getInequalityConstraintValues(C_M, *pSet)) {
         delete pSet;
         return false;
       }
       J_i_col = (C_P - C_M) * one_over_two_delta;

       // reset delta
       dp(i) = 0.0;

       //each vector is a column vector of the hessian, so copy it in place...
       for (int j=0;j<nConstraints;j++) {
         if (!IS_ZERO(J_i_col(j))) {
           tripletList.push_back(SMTriplet(j, i, J_i_col(j)));
         }
       }
     }
     jacobian.setFromTriplets(tripletList.begin(), tripletList.end());
     delete pSet;
   }
   return true;
}

bool NonlinearFunctionConstraints::estimateLocalInequalityConstraintHessian(SparseMatrix& hessian, const Parameterization& params, int iConstraint) {
  // this is a very slow method that evaluates the hessian of the objective function through FD...
   SMTriplets hessianTriplets;
   numopt_common::SparseMatrix jacobian;
   Parameterization* pSet = params.clone();

   double delta = 10e-6;
   const double one_over_two_delta = 1.0 / (2.0 * delta);
   const int nLocalParams = params.getLocalSize();

   Vector C_P(nLocalParams), C_M(nLocalParams), H_i_col(nLocalParams);

   Vector dp = Vector::Zero(nLocalParams);

   for (int i=0;i<nLocalParams;i++) {
     dp(i) = delta;
     pSet->plus(pSet->getParams(), params.getParams(), dp);
     if (!getLocalInequalityConstraintJacobian(jacobian, *pSet)) {
       delete pSet;
       return false;
     }
     C_P = jacobian.toDense().row(iConstraint).transpose();


     dp(i) = -delta;
     pSet->plus(pSet->getParams(), params.getParams(), dp);
     if (!getLocalInequalityConstraintJacobian(jacobian, *pSet)) {
       delete pSet;
       return false;
     }
     C_M = jacobian.toDense().row(iConstraint).transpose();


     // reset delta
     dp(i) = 0.0;

     //and compute the row of the hessian
     H_i_col = (C_P - C_M) * one_over_two_delta;


     //each vector is a column vector of the hessian, so copy it in place...
     for (int j = 0;j < nLocalParams;j++) {
       if (!IS_ZERO(H_i_col(j))) {
         hessianTriplets.push_back(SMTriplet(j, i, H_i_col(j)));
       }
     }
   }
   hessian.resize(nLocalParams, nLocalParams);
   hessian.setFromTriplets(hessianTriplets.begin(), hessianTriplets.end());
   delete pSet;
   return true;
}

void NonlinearFunctionConstraints::checkDimensions(double nParameters) {
//  const ParameterizationIdentity p(nParameters);
//
//  //-- equality constraints
//  SparseMatrix eqConJacobian;
//  getEqualityConstraintsJacobian(eqConJacobian, p);
//  Vector eqConTargetValues;
//  getEqualityConstraintsTargetValues(eqConTargetValues);
//
//  NUMOPT_ASSERT_EQ(std::range_error, eqConTargetValues.size(), eqConJacobian.rows(), "Vector b has wrong size.");
//  if (eqConJacobian.size() > 0) {
//    NUMOPT_ASSERT_EQ(std::range_error, eqConJacobian.cols(), nParameters, "Matrix A has wrong size.");
//  }
//  //--
//
//  //-- inequality constraints
//  ineqConJacobian_ = getInequalityConstraintsJacobian(p);
//  ineqConMinValues_ = getInequalityConstraintsMinValues();
//  ineqConMaxValues_ = getInequalityConstraintsMaxValues();
//
//  if (ineqConJacobian_.size() > 0) {
//    NUMOPT_ASSERT_EQ(std::range_error, ineqConJacobian_.cols(), nParameters, "Matrix C has wrong size.");
//    NUMOPT_ASSERT_EQ(std::range_error, ineqConMinValues_.size(), ineqConJacobian_.rows(), "Vector d has wrong size.");
//    NUMOPT_ASSERT_EQ(std::range_error, ineqConMaxValues_.size(), ineqConJacobian_.rows(), "Vector f has wrong size.");
//  }
//  //--
//
//  //-- bounds
//  minBounds_ = getBoundConstraintsMinValues();
//  maxBounds_ = getBoundConstraintsMaxValues();
//  NUMOPT_ASSERT_EQ(std::range_error, minBounds_.size(), nParameters, "Vector l has wrong size.");
//  NUMOPT_ASSERT_EQ(std::range_error, maxBounds_.size(), nParameters, "Vector u has wrong size.");
//  //--
}


void NonlinearFunctionConstraints::printConstraintErrors(const Parameterization& params, double eqTol, double iqTol)
{
  if (getNumberOfEqualityConstraints() > 0) {
    Vector de(getNumberOfEqualityConstraints());
    getEqualityConstraintValues(de, params);

    Vector dtargets;
    getEqualityConstraintTargetValues(dtargets);
    Vector::Index maxIndex;
    double maxError = (de-dtargets).cwiseAbs().maxCoeff(&maxIndex);
    if (maxError > eqTol) {
      printf("-----> Max equality constraint error: %10.10lf at index %u\n", maxError, static_cast<unsigned int>(maxIndex));
    }
    else {
      printf("   Equality constraints are within the tolerance.\n");
    }
  }

  if (getNumberOfInequalityConstraints() > 0) {
    Vector de(getNumberOfInequalityConstraints());
    getInequalityConstraintValues(de, params);

    Vector::Index maxIndexMin;
    Vector::Index maxIndexMax;
    Vector values(getNumberOfInequalityConstraints());
    getInequalityConstraintMinValues(values);
    double maxErrorMin = ((-de+values).array().max(Eigen::ArrayXd::Zero(getNumberOfInequalityConstraints()))).maxCoeff(&maxIndexMin);
    getInequalityConstraintMaxValues(values);
    double maxErrorMax = ((de-values).array().max(Eigen::ArrayXd::Zero(getNumberOfInequalityConstraints()))).maxCoeff(&maxIndexMax);
    Vector::Index maxIndex;
    double maxError = 0.0;
    if (maxErrorMin > maxErrorMax) {
      maxError = maxErrorMin;
      maxIndex = maxIndexMin;
    }
    else {
      maxError = maxErrorMax;
      maxIndex = maxIndexMax;
    }
    if (maxError > iqTol) {
      printf("------> Max inequality constraint error: %10.10lf at index %u\n", maxError, static_cast<unsigned int>(maxIndex));
    }
    else {
      printf("   Inequality constraints are within the tolerance.\n");
    }
  }

  Vector minVals(params.getGlobalSize());
  getGlobalBoundConstraintMinValues(minVals);
  Vector maxVals(params.getGlobalSize());
  getGlobalBoundConstraintMaxValues(maxVals);

  if (minVals.size() == maxVals.size() && minVals.size() == params.getGlobalSize()){
    for (uint i=0;i<params.getGlobalSize();i++) {
      if (minVals(i) != maxVals(i) && ( params.getParams()(i) < minVals(i) || params.getParams()(i) > maxVals(i))) {
        printf("-------> Error: Bound %d: %lf < %lf < %lf\n", i, minVals(i), params.getParams()(i), maxVals(i));
      }
    }
  }

  printf("\n");
}

} // namespace sooqp

