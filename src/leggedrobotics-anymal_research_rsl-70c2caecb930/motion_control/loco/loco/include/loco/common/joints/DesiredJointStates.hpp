/*
 * DesiredJointStates.hpp
 *
 *  Created on: Feb 13, 2017
 *      Author: Gabriel Hottiger
 */

#pragma once

#include "loco/common/joints/JointStates.hpp"

namespace loco {

class DesiredJointStates : public JointStates {
 public:
  explicit DesiredJointStates(const unsigned int nJoints);
  ~DesiredJointStates() override = default;
};

} /* namespace loco */
