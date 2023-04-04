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
* @file    ParameterizationQuaternionProblem.cpp
* @author  Christian Gehring
* @date    June, 2016
*/

#include <gtest/gtest.h>
#include <numopt_common/numopt_gtest.hpp>
#include <numopt_common/ParameterizationIdentity.hpp>
#include "numopt_problems/ParameterizationQuaternion.hpp"
#include "numopt_problems/RotationProblem.hpp"

TEST(ParameterizationQuaternionProblem, plus_zero)
{
  using namespace numopt_problems;
  using namespace numopt_common;
  typedef ParameterizationQuaternion::RotationQuaternion Quaternion;
  typedef kindr::EulerAnglesZyxPD EulerAnglesZyx;

  ParameterizationQuaternion params;
  Params p = Quaternion(EulerAnglesZyx(M_PI/2.0, 0.0, 0.0)).vector();
  Params delta = Eigen::Vector3d::Zero();
  Params pPlusDelta = Quaternion(EulerAnglesZyx(0.0, 0.3, 0.0)).vector();
  ASSERT_TRUE(params.plus(pPlusDelta, p, delta));
  NUMOPT_ASSERT_DOUBLE_MX_EQ(p, pPlusDelta, 1e-3, "plus_zero");

}

TEST(ParameterizationQuaternionProblem, plus_away_from_zero)
{
  using namespace numopt_problems;
  using namespace numopt_common;
  typedef ParameterizationQuaternion::RotationQuaternion Quaternion;
  typedef kindr::EulerAnglesZyxPD EulerAnglesZyx;

  ParameterizationQuaternion params;
  Params p = Quaternion(EulerAnglesZyx(M_PI/2.0, 0.0, 0.0)).vector();
  Params delta = Eigen::Vector3d(0.0, 0.0, M_PI/2.0);
  Params pPlusDelta = Quaternion(EulerAnglesZyx(0.0, 0.3, 0.0)).vector();

  Params pExpected = Quaternion(EulerAnglesZyx(M_PI, 0.0, 0.0)).vector();
  ASSERT_TRUE(params.plus(pPlusDelta, p, delta));
  NUMOPT_ASSERT_DOUBLE_MX_EQ(pExpected, pPlusDelta, 1e-3, "plus_away_from_zero");

}


