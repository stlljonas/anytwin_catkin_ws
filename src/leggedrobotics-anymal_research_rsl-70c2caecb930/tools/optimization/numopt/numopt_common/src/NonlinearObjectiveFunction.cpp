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
* @file    ObjectiveFunction.cpp
* @author  Stelian Coros, Christian Gehring
* @date    Aug 16, 2015
*/

#include "numopt_common/NonlinearObjectiveFunction.hpp"
#include <Eigen/Sparse>
#include <iostream>

namespace numopt_common {

NonlinearObjectiveFunction::NonlinearObjectiveFunction() :
    ObjectiveFunction()
{

}

NonlinearObjectiveFunction::~NonlinearObjectiveFunction() {

}

bool NonlinearObjectiveFunction::getLocalGradient(Vector& gradient, const Parameterization& p, bool newParams) {
  return estimateLocalGradient(gradient, p);
}

bool NonlinearObjectiveFunction::getGlobalGradient(Vector& gradient, const Parameterization& p, bool newParams) {
  Vector localGradient;
  if (!getLocalGradient(localGradient, p, newParams)) {
    return false;
  }
  return p.transformLocalVectorToGlobalVector(gradient, p.getParams(), localGradient);
}

bool NonlinearObjectiveFunction::getLocalHessian(numopt_common::SparseMatrix& hessian, const Parameterization& p, bool newParams) {
  return estimateLocalHessian(hessian, p);
}

bool NonlinearObjectiveFunction::getGlobalHessian(numopt_common::SparseMatrix& hessian, const Parameterization& p, bool newParams) {
  numopt_common::SparseMatrix localHessian;
  if (!getLocalHessian(localHessian, p, newParams)) {
    return false;
  }
  numopt_common::SparseMatrix transformMatrix;
  if (!p.getTransformMatrixGlobalToLocal(transformMatrix, p.getParams())) {
    return false;
  }
  hessian = transformMatrix.transpose()*localHessian*transformMatrix;
  return true;
}


bool NonlinearObjectiveFunction::getLocalHessianTriplets(SMTriplets& triplets, const Parameterization& p, bool newParams) {
  return estimateLocalHessianTriplets(triplets, p);
}


bool NonlinearObjectiveFunction::estimateLocalHessian(SparseMatrix& hessian, const Parameterization& params, double delta) {
  SMTriplets triplets;
  if (!estimateLocalHessianTriplets(triplets, params, delta)) {
    return false;
  }
  hessian.resize(params.getLocalSize(), params.getLocalSize());
  hessian.setFromTriplets(triplets.begin(), triplets.end());
  return true;
}

bool NonlinearObjectiveFunction::estimateLocalGradient(Vector& gradient, const Parameterization& params, double delta){
  //this is a very slow method that evaluates the gradient of the objective function through FD...

  const int nLocalParams = params.getLocalSize();
  gradient.resize(nLocalParams);

  const double one_over_two_delta = 1.0 / (2.0 * delta);

  Parameterization* pSet = params.clone();

  Scalar f_P, f_M;

  // delta vector
  Vector dp = Vector::Zero(nLocalParams);

  for (int i=0; i<nLocalParams; i++) {
    dp(i) = delta;
    if (!pSet->plus(pSet->getParams(), params.getParams(), dp)) {
      delete pSet;
      return false;
    }
    if(!computeValue(f_P, *pSet)) {
      delete pSet;
      return false;
    }
//    std::cout << "estimateLocalGradient: f=" << f_P << " p+dp: " << pSet->getParams().transpose() << std::endl;

    dp(i) = -delta;
    if (!pSet->plus(pSet->getParams(), params.getParams(), dp)) {
      delete pSet;
      return false;
    }
    if(!computeValue(f_M, *pSet)) {
      delete pSet;
      return false;
    }
//    std::cout << "estimateLocalGradient: f=" << f_M << " p-dp: " << pSet->getParams().transpose() << std::endl;

    gradient(i) = (f_P - f_M) * one_over_two_delta;

    // reset delta
    dp(i) = 0.0;

  }

  delete pSet;
  return true;
}

bool NonlinearObjectiveFunction::estimateLocalHessianTriplets(SMTriplets& hessianTriplets, const Parameterization& params, double delta)
{
  // this is a very slow method that evaluates the hessian of the objective function through FD...
  hessianTriplets.clear();
  Parameterization* pSet = params.clone();

  const double one_over_two_delta = 1.0 / (2.0 * delta);
  const int nLocalParams = params.getLocalSize();

  Vector C_P(nLocalParams), C_M(nLocalParams), H_i_col(nLocalParams);

  Vector dp = Vector::Zero(nLocalParams);

  for (int i=0;i<nLocalParams;i++) {
     dp(i) = delta;
     pSet->plus(pSet->getParams(), params.getParams(), dp);
     if (!getLocalGradient(C_P, *pSet)) {
       delete pSet;
       return false;
     }

     dp(i) = -delta;
     pSet->plus(pSet->getParams(), params.getParams(), dp);
     if (!getLocalGradient(C_M, *pSet)) {
       delete pSet;
       return false;
     }

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
  return true;
}

} // namespace sooqp



