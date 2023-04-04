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
* @file    Parameterization.hpp
* @author  Christian Gehring
* @date     Jun 9, 2016
*/
#pragma once

#include "numopt_common/numopt_common.hpp"

namespace numopt_common {

//! Parameterization interface
class Parameterization {
 public:
  Parameterization();
  virtual ~Parameterization();

  /*! @returns the parameter vector (with size getGlobalSize())
   */
  virtual Params& getParams() = 0;

  /*! @returns the parameter vector (with size getGlobalSize())
   */
  virtual const Params& getParams() const = 0;

  /*! Plus operator to add a delta vector in local space to the parameter vector in global space.
   * @param result    resulting parameter vector (res = p + dp)
   * @param p         parameter vector
   * @param dp        delta vector
   * @returns true if successful
   */
  virtual bool plus(Params& result,
                   const Params& p,
                   const Delta& dp) const = 0;

 /*! Returns the transformation matrix from local to global
  * @param matrix     globalSize x localSize matrix
  * @param params     parameters
  * @return true if successful
  */
 virtual bool getTransformMatrixLocalToGlobal(numopt_common::SparseMatrix& matrix,
                                      const Params& params) const = 0;

 /*! Returns the transformation matrix from global to local
  * @param matrix     localSize x globalSize matrix
  * @param params     parameters
  * @returns true if successful
  */
 virtual bool getTransformMatrixGlobalToLocal(numopt_common::SparseMatrix& matrix,
                                              const Params& params) const = 0;

 /*! Transforms a local vector to a global vector
  *
  * @param[out] globalVector
  * @param params
  * @param localVector
  * @returns true if successful
  */
 virtual bool transformLocalVectorToGlobalVector(numopt_common::Vector& globalVector,
                                     const Params& params,
                                     const numopt_common::Vector& localVector) const {
   numopt_common::SparseMatrix matrix;
   if (!getTransformMatrixLocalToGlobal(matrix, params)) {
     return false;
   }
   globalVector = matrix*localVector;
   return true;
 }

 //! @returns the size of the parameter space (size of the parameter vector)
 virtual int getGlobalSize() const = 0;

 //! @returns the size of the local parameter space (size of the delta vector).
 virtual int getLocalSize() const = 0;

 /*! Fills the parameter vector with random values.
  * @param[out] params   parameter vector with random values
  * @returns   true if successful
  */
 virtual bool setRandom(Params& params) const = 0;

 /*! Sets the parameter vector to identity.
  * @param[out] params   parameter vector
  * @returns   true if successful
  */
 virtual bool setIdentity(Params& params) const = 0;

 /*! Clones this parameterization.
  * @returns reference to clone.
  */
 virtual Parameterization* clone() const = 0;
};

} /* namespace numopt_common */
