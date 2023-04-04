//
// Created by dbellicoso on 03/04/18.
//

#pragma once

// loco crawling ros
#include <anymal_ctrl_dynamic_gaits_ros/topology.hpp>

// stl
#include <string>

namespace anymal_ctrl_dynamic_gaits_ros {
namespace topology_conversions {

inline static constexpr const char * getKeyNameFromKeyEnum(ControllerMode mode) {
  return getControllerModeKeys()[mode].getName();
}

inline static const ControllerMode getKeyEnumFromKeyName(const std::string& name) {
  return getControllerModeKeys().atName(name).getEnum();
}

inline static bool isValidControllerMode(const std::string& name) {
  return getControllerModeKeys().containsName(name);
}



} /* namespace topology_conversions */
} /* namespace anymal_ctrl_dynamic_gaits_ros */
