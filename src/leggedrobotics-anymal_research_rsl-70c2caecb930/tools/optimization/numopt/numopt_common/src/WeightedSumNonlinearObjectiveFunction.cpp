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
 * WeightedSumObjectiveFunction.cpp
 *
 *  Created on: Jun 28, 2016
 *      Author: Christian Gehring
 */

#include "numopt_common/WeightedSumNonlinearObjectiveFunction.hpp"

namespace numopt_common {

WeightedSumNonlinearObjectiveFunction::WeightedSumNonlinearObjectiveFunction():objectives_()
{

}

WeightedSumNonlinearObjectiveFunction::WeightedSumNonlinearObjectiveFunction(const Objectives& objectives):objectives_(objectives) {

}

WeightedSumNonlinearObjectiveFunction::~WeightedSumNonlinearObjectiveFunction() {

}

bool WeightedSumNonlinearObjectiveFunction::addObjective(std::shared_ptr<Objective> objective) {
  objectives_.push_back(objective);
  return true;
}

void WeightedSumNonlinearObjectiveFunction::clear() {
  objectives_.clear();
}

bool WeightedSumNonlinearObjectiveFunction::computeValue(double& functionValue, const numopt_common::Parameterization& params, bool newParams) {
  functionValue = 0.0;

  double value;
  for (uint i=0; i<objectives_.size(); i++) {
    if(!objectives_[i]->computeValue(value, params, newParams)) {
      return false;
    }
    functionValue += objectives_[i]->getWeight() * value;
  }
  return true;
}

bool WeightedSumNonlinearObjectiveFunction::getLocalGradient(numopt_common::Vector& gradient, const numopt_common::Parameterization& params, bool newParams) {
  gradient.setZero(params.getLocalSize());

  numopt_common::Vector tmpGradient;
  for (uint i=0; i<objectives_.size(); i++) {
    if(!objectives_[i]->getLocalGradient(tmpGradient, params, newParams)) {
      return false;
    }
    gradient += objectives_[i]->getWeight()*tmpGradient;
  }
  return true;
}

bool WeightedSumNonlinearObjectiveFunction::getLocalHessian(numopt_common::SparseMatrix& hessian, const numopt_common::Parameterization& params, bool newParams) {
  hessian.resize(params.getLocalSize(), params.getLocalSize());

  numopt_common::SparseMatrix tmpHessian;

  for (uint i=0; i<objectives_.size(); i++) {
    if (!objectives_[i]->getLocalHessian(tmpHessian, params, newParams)) {
      return false;
    }
    hessian += objectives_[i]->getWeight()*tmpHessian;
  }
  return true;
}


} /* namespace numopt_common */
