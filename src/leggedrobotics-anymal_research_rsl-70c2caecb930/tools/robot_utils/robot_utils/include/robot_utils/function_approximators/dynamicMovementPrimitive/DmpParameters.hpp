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
#include <cstdio>
#include <sstream>
#include <string>

// local includes
#include "robot_utils/function_approximators/dynamicMovementPrimitive/constants.hpp"

namespace dmp {

class DmpParameters {
 public:
  friend class DiscreteMovementPrimitive;

  /*! Constructor
   */
  DmpParameters();

  /*! Destructor
   */
  ~DmpParameters();

  /*!
   * @return
   */
  bool initialize(const double samplingFrequency, const double teachingDuration, const double executionDuration, const double canSysCutoff,
                  const double alphaZ, const double betaZ);

  /*!
   */
  void print();

  /*!
   */
  std::string getInfoString();

 private:
  /*! Indicates whether the DMP has been learned, i.e. the parameters of the DMP has been computed/set
   */
  bool isLearned_ = false;

  /*! Indicates whether the DMP is setup, i.e. the start, goal, and duration has been set
   *
   */
  bool isSetUp_ = false;

  /*! Indicates whether the start of the DMP has been set. This flag is used to update the start position of
   *  successive DMPs, when the start position cannot be determined beforehand.
   */
  bool isStartSet_ = false;

  /*! Number of transformation systems (dimensions) used in the DMP
   */
  int numTransformationSystems_ = 0;

  /*! Timing parameters used during learning and integrating the system.
   *  Tau is the length of the trajectory
   */
  double tau_ = 0.0;
  double initialTau_ = 0.0;

  /*! Time step of the integration
   */
  double deltaT_ = 0.0;
  double initialDeltaT_ = 0.0;

  /*! Default durations are important to find values for alphaZ to obtain desired behavior
   */
  double teachingDuration_ = -1.0;
  double executionDuration_ = -1.0;

  /*! Remaining value of canonical system at end time (time = tau)
   */
  double canSysCutoff_ = 0.0;

  /*! time constant of the canonical system (defined by canSysCutoff_)
   */
  double alphaX_ = -1.0;

  /*! time constant of the transformation systems
   */
  double alphaZ_ = -1.0;
  double betaZ_ = -1.0;

  //! Number of the current sample
  int sampleIndex_ = 0;

  //! Internal sampling frequency of the DMP
  double defaultSamplingFrequency_ = 0.0;
};

}  // namespace dmp
