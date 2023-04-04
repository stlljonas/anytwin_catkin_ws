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
* @file    ConstrainedNonlinearProblemSolver.hpp
* @author  Stelian Coros, Christian Gehring
* @date    Aug 16, 2015
*/
#pragma once

#include <numopt_common/ConstrainedNonlinearProblem.hpp>
#include "numopt_common/ProblemSolver.hpp"

namespace numopt_common {

class ConstrainedNonlinearProblemSolver : public ProblemSolver
{
 public:
  ConstrainedNonlinearProblemSolver();
  ~ConstrainedNonlinearProblemSolver() override = default;

  /** @returns true if it successfully solved the constrained non-linear problem.
    min f(p) subject to the constraints...
    @param  problem    the constrained non-linear problem
    @param  initial parameters and best solution
    @param  minimal objective function value
  */
  virtual bool minimize(numopt_common::ConstrainedNonlinearProblem* problem,
      numopt_common::Parameterization& p,
      double &functionValue,
      const std::string& fileName = "",
      bool printProblemToFile = false) = 0;
};

} /* namespace numopt_common */

