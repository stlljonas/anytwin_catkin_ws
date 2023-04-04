/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Christian Gehring, Stelian Coros
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
* @file    numopt_math.hpp
* @author  Christian Gehring
* @date    Feb, 2016
*/

#pragma once

#include <Eigen/Cholesky>
#include <Eigen/SparseCholesky>

namespace numopt_common {

template <typename Derived>
bool isPositiveDefinite(const Eigen::DenseBase<Derived>& A) {
  /* The matrix A is positive definite if and only if there exists a
   * unique lower triangular matrix L, with real and strictly positive diagonal elements,
   * such that A = LL*. This factorization is called the Cholesky decomposition of A.
   */
  Eigen::LLT<Derived> lltOfA(A);
  return (lltOfA.info() == Eigen::ComputationInfo::Success);
}

template <typename Derived>
bool isPositiveDefinite(const Eigen::SparseMatrixBase<Derived>& A) {
  /* The matrix A is positive definite if and only if there exists a
   * unique lower triangular matrix L, with real and strictly positive diagonal elements,
   * such that A = LL*. This factorization is called the Cholesky decomposition of A.
   */
  Eigen::SimplicialLLT<Derived> lltOfA(A);
  return (lltOfA.info() == Eigen::ComputationInfo::Success);
}

}
