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
* @file    QuadraticObjectiveFunction.cpp
* @author  Christian Gehring, Dario Bellicoso
* @date    Aug 16, 2015
*/


#include "numopt_common/QuadraticObjectiveFunction.hpp"

namespace numopt_common {

QuadraticObjectiveFunction::QuadraticObjectiveFunction(const numopt_common::SparseMatrix& Q, const Vector& c)
{
  setGlobalHessian(Q);
  setLinearTerm(c);
}

bool QuadraticObjectiveFunction::computeValue(double& value, const Parameterization& p, bool newParams) {
  const double tmp =  linearTerm_.transpose()*p.getParams();
  double tmp2 = p.getParams().transpose()*globalHessian_*p.getParams();
  value = 0.5*tmp2 + tmp;
  return true;
}

void QuadraticObjectiveFunction::setGlobalHessian(const numopt_common::SparseMatrix& Q) {
  globalHessian_ = Q;
  setHessianTripletsFromMatrix(globalHessianTriplets_, globalHessian_);
}

void QuadraticObjectiveFunction::setLinearTerm(const Vector& c) {
  linearTerm_ = c;
}

bool QuadraticObjectiveFunction::getGlobalGradient(Vector& gradient, const Parameterization& p, bool newParams) {
  gradient = globalHessian_*p.getParams() + linearTerm_;
  return true;
}


bool QuadraticObjectiveFunction::getLinearTerm(Vector& linearTerm) {
  linearTerm = linearTerm_;
  return true;
}

bool QuadraticObjectiveFunction::getLocalGradient(Vector& gradient, const Parameterization& p, bool newParams) {
  numopt_common::SparseMatrix transformMatrix;
  if (!p.getTransformMatrixLocalToGlobal(transformMatrix, p.getParams())) {
    return false;
  }
  Vector globalGradient;
  if (!getGlobalGradient(globalGradient, p, newParams)) {
    return false;
  }
  gradient = transformMatrix*globalGradient;
  return true;
}

bool QuadraticObjectiveFunction::getGlobalHessian(numopt_common::SparseMatrix& hessian, const Parameterization& p, bool newParams) {
  // constant
  hessian = globalHessian_;
  return true;
}


bool QuadraticObjectiveFunction::getLocalHessian(numopt_common::SparseMatrix& Q, const Parameterization& p, bool newParams) {
  numopt_common::SparseMatrix transformMatrix;
  if (!p.getTransformMatrixLocalToGlobal(transformMatrix, p.getParams())) {
    return false;
  }
  Q = transformMatrix.transpose()*globalHessian_*transformMatrix;
  return true;
}

bool QuadraticObjectiveFunction::getGlobalHessianTriplets(SMTriplets& triplets, const Parameterization& p, bool newParams) {
  triplets = globalHessianTriplets_;
  return true;
}

void QuadraticObjectiveFunction::setHessianTripletsFromMatrix(SMTriplets& triplets, numopt_common::SparseMatrix& hessian) {
  triplets.clear();

  for (int k=0; k<hessian.outerSize(); ++k) {
    for (numopt_common::SparseMatrix::InnerIterator it(hessian,k); it; ++it) {
      triplets.push_back(SMTriplet(it.row(), it.col(), it.value()));
    }
  }
}


bool QuadraticObjectiveFunction::printToFile(std::ofstream& file) const {
  file << "[hessian] " << std::to_string((int)globalHessian_.rows()) << " " << std::to_string((int)globalHessian_.cols()) << "\n"
       << std::scientific << globalHessian_.toDense() << "\n";
  file << "[linear_term] " << std::to_string((int)linearTerm_.size()) << " " << std::to_string(1) << "\n"
       << std::scientific << linearTerm_  << "\n";
  return true;
}

} /* namespace numopt_common */
