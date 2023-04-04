/*
 * topology_conversions.hpp
 *
 *  Created on: Mar 21, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include <loco/common/topology.hpp>
#include <loco/common/typedefs.hpp>

// stl
#include <string>

namespace loco {
namespace topology_conversions {

inline static constexpr const char* getControlModeStringFromControlModeEnum(loco::ControlMode mode) {
  return loco::getControlModeKeys()[mode].getName();
}

inline static const loco::ControlMode getControlModeEnumFromControlModeString(const std::string& name) {
  return loco::getControlModeKeys().atName(name).getEnum();
}

} /* namespace topology_conversions */
} /* namespace loco */
