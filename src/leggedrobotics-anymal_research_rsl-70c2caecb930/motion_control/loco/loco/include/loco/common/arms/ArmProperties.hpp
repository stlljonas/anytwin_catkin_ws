/*
 * ArmProperties.hpp
 *
 *  Created on: Mar 19, 2018
 *      Author: Koen Krämer
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

// loco
#include "loco/common/limbs/LimbProperties.hpp"
#include "loco/common/typedefs.hpp"

// STL
#include <memory>

namespace loco {

class ArmProperties : public LimbProperties {
 public:
  ArmProperties() = default;
  ~ArmProperties() override = default;
};

using ArmPropertiesPtr = std::unique_ptr<ArmProperties>;

} /* namespace loco */
