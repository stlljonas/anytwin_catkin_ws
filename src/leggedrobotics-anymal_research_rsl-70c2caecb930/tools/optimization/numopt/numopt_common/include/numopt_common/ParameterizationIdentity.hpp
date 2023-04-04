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
 * ParameterizationIdentity.hpp
 *
 *  Created on: Jun 9, 2016
 *      Author: Christian Gehring
 */

#pragma once

#include <numopt_common/Parameterization.hpp>

namespace numopt_common {

//! Identity Parameterization
/*! Local space is equal to global space.
 *
 */
class ParameterizationIdentity : public Parameterization {
 public:
  explicit ParameterizationIdentity(int size = 0);
  explicit ParameterizationIdentity(const Params& p);
  ~ParameterizationIdentity() override = default;

  bool plus(Params& result,
            const Params& p,
            const Delta& dp) const override;

  /*! Returns the transformation matrix from local to global
   * @param matrix     globalSize x localSize matrix
   * @param params     parameters
   * @return true if successful
   */
  bool getTransformMatrixLocalToGlobal(numopt_common::SparseMatrix& matrix,
                                       const Params& params) const override;

  /*! Returns the transformation matrix from global to local
   * @param matrix     localSize x globalSize matrix
   * @param params     parameters
   * @return true if successful
   */
  bool getTransformMatrixGlobalToLocal(numopt_common::SparseMatrix& matrix,
                                       const Params& params) const override;


  bool transformLocalVectorToGlobalVector(numopt_common::Vector& globalVector,
                                          const Params& params,
                                          const numopt_common::Vector& localVector) const override;

  // Size of delta.
  int getLocalSize() const override {
    return params_.size();
  }

  // Size of p.
  int getGlobalSize() const override {
    return params_.size();
  }

  Params& getParams() override {
    return params_;
  }

  const Params& getParams() const override {
    return params_;
  }

  Parameterization* clone() const override {
    Parameterization* clone = new ParameterizationIdentity(*this);
    return clone;
  }

  bool setRandom(Params& p) const override;
  bool setIdentity(Params& p) const override;

 protected:
  // This is the (global) parameter vector of size getGlobalSize()
  Params params_;
};

} /* namespace numopt_common */

