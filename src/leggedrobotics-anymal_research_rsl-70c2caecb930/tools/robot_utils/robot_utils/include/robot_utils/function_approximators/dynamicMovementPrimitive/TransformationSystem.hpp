/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

 \author  Peter Pastor, Peter Fankhauser

 **********************************************************************/

#pragma once

// system includes
#include <Eigen/Eigen>
#include <algorithm>
#include <boost/foreach.hpp>
#include <cassert>
#include <cerrno>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

// local includes
#include "robot_utils/function_approximators/dynamicMovementPrimitive/DiscreteMovementPrimitive.hpp"
#include "robot_utils/function_approximators/dynamicMovementPrimitive/GaussianKernel.hpp"
#include "robot_utils/function_approximators/dynamicMovementPrimitive/constants.hpp"

namespace dmp {

class DiscreteMovementPrimitive;
class GaussianKernel;

class TransformationSystem {
 public:
  friend class DiscreteMovementPrimitive;
  friend class GaussianKernel;

  /*!
   */
  explicit TransformationSystem(DiscreteMovementPrimitive* dmpParent);

  /*!
   */
  ~TransformationSystem();

  /*!
   *
   * @param index
   * @param directory_name
   * @param lwr_params
   * @return
   */
  // bool initialize(std::string library_directory_name, int dmp_id, int transId, lwr::Parameters lwr_params);
  bool initialize(const int numRfs, const double activation, const bool exponentiallySpaced, const double canSysCutoff);

  /*!
   */
  void print();

  /*!
   * @return
   */
  std::string getInfoString();

 private:
  DiscreteMovementPrimitive* dmpParent_;

  /*! flag that indicates whether the system is initialized
   *
   */
  bool initialized_;

  /*! the id of the transformation system
   *
   */
  int transId_;

  /*! determines which DMP version is used. (not used yet)
   *
   */
  // int versionId_;

  /*!
   */
  void reset();

  /*!
   */
  void setState(const double y, const double z);
  double getState() const;

  /*!
   */
  void setStart(const double y0);
  void setGoal(const double goal);
  double getGoal() const;

  /*!
   */
  void setInitialStart(const double initialY0);
  void setInitialGoal(const double initialGoal);

  /*!
   */
  void resetMSE();
  bool getMSE(double& mse);
  bool getNormalizedMSE(double& normalized_mse);
  void computeMSE();

  /*! internal states
   *
   */
  double z_;   // vel
  double zd_;  // acc

  /*! external states
   *
   */
  double y_;    // pos
  double yd_;   // vel
  double ydd_;  // acc

  /*! the targets used during supervised learning
   *
   */
  double t_;
  double td_;
  double tdd_;

  /*! the start state
   *
   */
  double y0_;
  double initialY0_;

  /*! the goal state
   *
   */
  double goal_;
  double initialGoal_;

  /*! the current values of the nonlinear function
   *
   */
  double f_;
  double ft_;

  /*! internal variables that are used to compute the normalized mean squared
   *  error during learning
   */
  double mse_;
  double meanFt_;
  unsigned int numMseDataPoints_;

  /*! internal variable that is used to store the target function for Gaussian kernel model
   *
   */
  // std::vector<double> trajectoryTarget_;

  /*! gaussian kernel model used to approximate the nonlinear function
   *
   */
  GaussianKernel* gaussianKernelModel_;
};

// inline function follow
inline void TransformationSystem::reset() {
  z_ = 0.0;
  zd_ = 0.0;
  y_ = 0.0;
  yd_ = 0.0;
  ydd_ = 0.0;
}
inline void TransformationSystem::setState(const double y, const double z) {
  y_ = y;
  z_ = z;
}
inline double TransformationSystem::getState() const {
  return y_;
}
inline void TransformationSystem::setStart(const double y0) {
  y0_ = y0;
}
inline void TransformationSystem::setGoal(const double goal) {
  goal_ = goal;
}
inline double TransformationSystem::getGoal() const {
  return goal_;
}
inline void TransformationSystem::setInitialStart(const double initialY0) {
  initialY0_ = initialY0;
}
inline void TransformationSystem::setInitialGoal(const double initialGoal) {
  initialGoal_ = initialGoal;
}
inline void TransformationSystem::resetMSE() {
  mse_ = 0.0;
  meanFt_ = 0.0;
  numMseDataPoints_ = 0;
}
inline bool TransformationSystem::getMSE(double& mse) {
  if (numMseDataPoints_ == 0) {
    printf("No data point seen yet, cannot compute mean squared error.");
    return false;
  }
  mse = mse_ / static_cast<double>(numMseDataPoints_);
  return true;
}
inline bool TransformationSystem::getNormalizedMSE(double& normalized_mse) {
  if (numMseDataPoints_ == 0) {
    printf("No data point seen yet, cannot compute normalized mean squared error.");
    return false;
  }

  normalized_mse = (static_cast<double>(1.0) / std::max(pow(meanFt_, 2), 1.0)) * (mse_ / static_cast<double>(numMseDataPoints_));
  return true;
}
inline void TransformationSystem::computeMSE() {
  mse_ += pow(ft_ - f_, 2);
  meanFt_ = (static_cast<double>(numMseDataPoints_) * meanFt_ + ft_) / static_cast<double>(numMseDataPoints_ + 1);
  numMseDataPoints_++;
}

}  // namespace dmp
