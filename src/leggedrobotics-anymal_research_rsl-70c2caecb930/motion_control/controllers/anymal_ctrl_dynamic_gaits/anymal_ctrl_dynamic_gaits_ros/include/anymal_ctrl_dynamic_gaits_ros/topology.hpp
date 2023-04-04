//
// Created by dbellicoso on 03/04/18.
//

#pragma once

// std utils
#include <std_utils/std_utils.hpp>

namespace anymal_ctrl_dynamic_gaits_ros {

CONSECUTIVE_ENUM(ControllerMode, Walk, Stand)

namespace internal {
static constexpr std_utils::KeyArray<ControllerMode> controllerModeKeys{{
  std_utils::make_key(ControllerMode::Walk,  "walk"),
  std_utils::make_key(ControllerMode::Stand, "stand")
}};
} /* namespace internal */

inline static constexpr const std_utils::KeyArray<ControllerMode> getControllerModeKeys() {
  return internal::controllerModeKeys;
}

} /* namespace anymal_ctrl_dynamic_gaits_ros */
