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
/*
 * L2Regularizer.cpp
 *
 *  Created on: Jun 28, 2016
 *      Author: Christian Gehring
 */

#include "numopt_common/L2Regularizer.hpp"

namespace numopt_common {

L2Regularizer::L2Regularizer(int nParameters, double regularizer) :
    nParameters_(nParameters),
    regularizer_(regularizer)
{
  setLinearTerm(Vector::Zero(nParameters_));
  SparseMatrix Q;
  Q.resize(nParameters, nParameters);
  globalHessianTriplets_.clear();
  for (int i=0;i<nParameters_;i++) {
    globalHessianTriplets_.push_back(SMTriplet(i, i, regularizer_));
  }
  Q.setFromTriplets(globalHessianTriplets_.begin(), globalHessianTriplets_.end());
  setGlobalHessian(Q);
}

L2Regularizer::~L2Regularizer() {

}

} /* namespace numopt_common */
