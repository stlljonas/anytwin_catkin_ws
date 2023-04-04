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

// local includes
#include "robot_utils/function_approximators/dynamicMovementPrimitive/DmpParameters.hpp"

namespace dmp {

DmpParameters::DmpParameters() = default;

DmpParameters::~DmpParameters() = default;

bool DmpParameters::initialize(const double samplingFrequency, const double teachingDuration, const double executionDuration,
                               const double canSysCutoff, const double alphaZ, const double betaZ) {
  if (teachingDuration <= 0) {
    printf("Teaching duration is invalid (%.1f sec).", teachingDuration);
    return false;
  }

  if (executionDuration <= 0) {
    printf("Execution duration is invalid (%.1f sec).", executionDuration);
    return false;
  }

  if (canSysCutoff <= 0) {
    printf("Canonical system cutoff frequency is invalid (%.1f).", canSysCutoff);
    return false;
  }

  if (alphaZ <= 0) {
    printf("Time constant alphaZ is invalid (%f).", alphaZ);
    return false;
  }

  if (betaZ <= 0) {
    printf("Time constant betaZ is invalid (%f).", betaZ);
    return false;
  }

  teachingDuration_ = teachingDuration;
  executionDuration_ = executionDuration;
  canSysCutoff_ = canSysCutoff;
  alphaZ_ = alphaZ;
  betaZ_ = betaZ;
  defaultSamplingFrequency_ = samplingFrequency;

  return true;
}

void DmpParameters::print() {
  // printf("%s", getInfoString());
}

std::string DmpParameters::getInfoString() {
  std::string info;
  std::stringstream ss;

  int precision = 3;
  ss.precision(precision);
  ss << std::fixed;

  ss << teachingDuration_;
  info.append(std::string("\t"));
  info.append(std::string("teaching duration: ") + ss.str());
  ss.str("");
  ss.clear();

  ss << executionDuration_;
  info.append(std::string("   execution duration: ") + ss.str());
  ss.str("");
  ss.clear();

  info.append(std::string("\n\t"));
  info.append(std::string("canonical system parameters>      "));

  ss << alphaX_;
  info.append(std::string("  alpha_x: ") + ss.str());
  ss.str("");
  ss.clear();

  ss << canSysCutoff_;
  info.append(std::string("   canonical system cutoff: ") + ss.str());
  ss.str("");
  ss.clear();

  info.append(std::string("\n\t"));
  info.append(std::string("transformation system parameters> "));

  ss << alphaZ_;
  info.append(std::string("   alphaZ: ") + ss.str());
  ss.str("");
  ss.clear();

  ss << betaZ_;
  info.append(std::string("   betaZ: ") + ss.str());
  ss.str("");
  ss.clear();

  return info;
}

}  // namespace dmp
