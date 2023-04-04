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
/* ParameterizationQuaternion.hpp
 *
 *  Created on: Jun 11, 2016
 *      Author: gech
 */

#pragma once

#include <numopt_common/Parameterization.hpp>
#include <kindr/Core>

namespace numopt_problems {


//! Example Parameterization of an orientation of a body.
/*!
 * The parameter vector encodes the orientation of the boy in form of
 * a Hamiltonian unit quaternion:
 *  p(0) = q.w
 *  p(1) = q.x
 *  p(2) = q.y
 *  p(3) = q.z
 *
 * The local manifold can be described by the local angular velocity of the body:
 *  u(0) = w.x
 *  u(1) = w.y
 *  u(2) = w.z
 */
class ParameterizationQuaternion : public numopt_common::Parameterization {
 public:
  typedef kindr::RotationQuaternionD RotationQuaternion;
 public:
  ParameterizationQuaternion():params_(4) {

  }
  ParameterizationQuaternion(const ParameterizationQuaternion& other): params_(other.params_) {

  }

  virtual ~ParameterizationQuaternion() {

  }

  virtual numopt_common::Params& getParams() {
    return params_;
  }

  virtual const numopt_common::Params& getParams() const {
    return params_;
  }

  // p := q_IB
  // dp := I_w_IB*dt
  // p_k1 = pk + dp
  virtual bool plus(numopt_common::Params& result,
                   const numopt_common::Params& p,
                   const numopt_common::Delta& dp) const {
    const RotationQuaternion quat_IB(p);
    result = quat_IB.boxPlus(Eigen::Vector3d(dp)).vector();
    return true;
  }



  /*! Returns the transformation matrix from local to global
   * @param matrix     globalSize x localSize matrix
   * @param params     parameters
   * @return true if successful
   */
  virtual bool getTransformMatrixLocalToGlobal(numopt_common::SparseMatrix& matrix,
                                       const numopt_common::Params& params) const  {
    matrix = (0.5*RotationQuaternion(params).getLocalQuaternionDiffMatrix().transpose()).sparseView();
    return true;
  }

  /*! Returns the transformation matrix from global to local
   * @param matrix     localSize x globalSize matrix
   * @param params     parameters
   * @return true if successful
   */
  virtual bool getTransformMatrixGlobalToLocal(numopt_common::SparseMatrix& matrix,
                                               const numopt_common::Params& params) const {

    matrix = (2.0*RotationQuaternion(params).getLocalQuaternionDiffMatrix()).sparseView();
    return true;
  }


  // Size of p.
  virtual int getGlobalSize() const {
   return 4;
  }

  // Size of dp (delta).
  virtual int getLocalSize() const {
   return 3;
  }

  virtual Parameterization* clone() const {
    Parameterization* clone = new ParameterizationQuaternion(*this);
    return clone;
  }

  bool setRandom(numopt_common::Params& p) const {
    p.setRandom(4);
    p.normalize();
    return true;
  }

  bool setIdentity(numopt_common::Params& p) const {
    p.setZero(4);
    p(0) = 1.0;
    return true;
  }

 protected:
  numopt_common::Params params_;
};

} /* namespace numopt_problems */
