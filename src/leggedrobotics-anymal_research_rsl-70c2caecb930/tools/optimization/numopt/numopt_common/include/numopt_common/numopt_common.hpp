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
* @file    common.hpp
* @author  Christian Gehring
* @date    Aug 16, 2015
*/
#pragma once

#include <Eigen/SparseCore>
#include <vector>
#include <limits>
#include <cfloat>

#define MYEPSILON 1e-90
//#define EPSILON 0.0000000001
#define IS_ZERO(x) ZERO_WITHIN_EPSILON(x)
#define ZERO_WITHIN_EPSILON(x) ((x)>-MYEPSILON && (x)<MYEPSILON)
//#define DBL_MIN -std::numeric_limits<double>::max()

namespace numopt_common {

typedef double Scalar;
typedef Eigen::SparseMatrix<Scalar, Eigen::RowMajor> SparseMatrix;
typedef Eigen::Triplet<Scalar> SMTriplet;
typedef std::vector<SMTriplet> SMTriplets;
typedef Eigen::VectorXd Params;
typedef Eigen::VectorXd Delta;
typedef Eigen::VectorXd Vector;

enum class QuadraticProgrammingAlgorithmEnum : unsigned int {
  InteriorPoint = 0,
  ActiveSet,
  ParametricActiveSet,
  AlternatingDirectionMethodOfMultipliers,
  Undefined
};

enum class NonlinProgrammingAlgorithmEnum : unsigned int {
  SequentialQuadratic = 0,
  InteriorPoint,
  Undefined
};

} /* namespace numopt_common */
