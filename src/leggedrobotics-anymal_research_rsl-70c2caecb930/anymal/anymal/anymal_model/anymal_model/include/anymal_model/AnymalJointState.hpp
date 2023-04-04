#pragma once

// anymal model
#include "anymal_model/typedefs.hpp"

// any measurements
#include <any_measurements/ExtendedJointState.hpp>

// std utils
#include <std_utils/containers/EnumArray.hpp>

namespace anymal_model {

using AnymalJointState = std_utils::EnumArray<AD::JointEnum, any_measurements::ExtendedJointState>;

}  // namespace anymal_model
