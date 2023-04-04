/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, C. Dario Bellicoso, Christian Gehring, Ralf Kaestner
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
 *   * Neither the name of Autonomous Systems Lab nor ETH Zurich
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
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
 * ParameterValueInterface.hpp
 *
 *  Created on: Sep 24, 2015
 *      Author: Christian Gehring, Ralf Kaestner
 */

#pragma once

#include <algorithm>
#include <memory>
#include <vector>

#include <parameter_handler/ParameterObserverInterface.hpp>

namespace parameter_handler {
namespace internal {

class ParameterValueInterface;

using ParameterValuePtr = std::shared_ptr<ParameterValueInterface>;

class ParameterValueInterface {
 public:
  ParameterValueInterface() = default;
  virtual ~ParameterValueInterface() = default;

  virtual ParameterValuePtr clone() const = 0;

  void addObserver(ParameterObserverInterface* observer) { observers_.push_back(observer); }

  void removeObserver(ParameterObserverInterface* observer) {
    auto it = std::find(observers_.begin(), observers_.end(), observer);
    if (it != observers_.end()) {
      observers_.erase(it);
    }
  }

  const std::vector<ParameterObserverInterface*>& getObservers() { return observers_; }

 private:
  std::vector<ParameterObserverInterface*> observers_;
};

}  // namespace internal
}  // namespace parameter_handler
