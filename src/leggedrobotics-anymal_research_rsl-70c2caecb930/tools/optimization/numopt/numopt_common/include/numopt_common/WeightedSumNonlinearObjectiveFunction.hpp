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
 * WeightedSumObjectiveFunction.hpp
 *
 *  Created on: Jun 28, 2016
 *      Author: Christian Gehring
 */

#pragma once

#include <numopt_common/NonlinearObjectiveFunction.hpp>
#include <vector>
#include <memory>

namespace numopt_common {

/*! This class implements an objective function which is the weighted sum of other non-linear objectives.
 *
 * f(p) = Sum_i w_i*f_i(p)
 */
class WeightedSumNonlinearObjectiveFunction: public NonlinearObjectiveFunction {
 public:
  typedef NonlinearObjectiveFunction Objective;
  typedef std::vector<std::shared_ptr<Objective>> Objectives;
 public:
  WeightedSumNonlinearObjectiveFunction();
  WeightedSumNonlinearObjectiveFunction(const Objectives& objectives);
  virtual ~WeightedSumNonlinearObjectiveFunction();
  bool addObjective(std::shared_ptr<Objective> objective);
  void clear();

  virtual bool computeValue(double& functionValue, const numopt_common::Parameterization& params, bool newParams = true);
  virtual bool getLocalGradient(numopt_common::Vector& gradient, const numopt_common::Parameterization& params, bool newParams = true);
  virtual bool getLocalHessian(numopt_common::SparseMatrix& hessian, const numopt_common::Parameterization& params, bool newParams = true);

 protected:
  Objectives objectives_;
};

} /* namespace numopt_common */
