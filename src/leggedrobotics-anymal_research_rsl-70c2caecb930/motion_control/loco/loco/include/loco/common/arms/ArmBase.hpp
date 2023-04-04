/*
 * ArmBase.hpp
 *
 *  Created on: Jan 9, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// loco
#include "loco/common/end_effectors/EndEffectorBase.hpp"
#include "loco/common/limbs/LimbBase.hpp"
#include "loco/common/limbs/LimbLinkGroup.hpp"
#include "loco/common/typedefs.hpp"

// kindr
#include <kindr/Core>

// eigen
#include <Eigen/Core>

// stl
#include <iostream>
#include <string>

namespace loco {

//! Base class for an arm
/*! This should be used only as a data container
 *
 */
class ArmBase : public LimbBase {
 public:
  ArmBase(const std::string& name, const unsigned int numDofLimb, LimbPropertiesPtr&& properties, EndEffectorBasePtr&& endeffector);

  ArmBase(const std::string& name, const unsigned int numDofLimb, LimbPropertiesPtr&& properties, EndEffectorBasePtr&& endeffector,
          LimbStateMeasuredPtr&& stateMeasured, LimbStateDesiredPtr&& stateDesired);

  ~ArmBase() override = default;

  void print(std::ostream& out) const override;
  bool addVariablesToLog(const std::string& ns) const override;

 private:
  //*! Namespace for logging.
  std::string logNameSpace_;
};

} /* namespace loco */
