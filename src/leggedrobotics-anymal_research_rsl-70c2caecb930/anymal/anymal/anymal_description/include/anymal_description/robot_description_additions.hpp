/*!
 * @file     robot_description_additions.hpp
 * @author   Dario Bellicoso
 * @date     Nov 6, 2017
 */

#pragma once

// anymal description
#include "anymal_description/AnymalTopology.hpp"

// romo
#include <romo/common/RobotDescription.hpp>

// std_utils
#include <std_utils/std_utils.hpp>

namespace romo {
namespace internal {

template <>
struct KeysHelper<anymal_description::AnymalTopology, typename anymal_description::AnymalTopology::FrameTransformEnum> {
  inline static constexpr std_utils::KeyArray<typename anymal_description::AnymalTopology::FrameTransformEnum> keys() {
    return anymal_description::AnymalTopology::frameTransformKeys;
  }
};

template <>
struct KeysHelper<anymal_description::AnymalTopology, typename anymal_description::AnymalTopology::FootEnum> {
  inline static constexpr std_utils::KeyArray<typename anymal_description::AnymalTopology::FootEnum> keys() {
    return anymal_description::AnymalTopology::footKeys;
  }
};

template <>
struct MapEnumHelper<typename anymal_description::AnymalTopology::FootEnum, typename anymal_description::AnymalTopology::BranchEnum,
                     anymal_description::AnymalTopology> {
  static constexpr typename anymal_description::AnymalTopology::BranchEnum map(
      typename anymal_description::AnymalTopology::FootEnum footEnum) {
    return anymal_description::AnymalTopology::mapFootEnumToBranchEnum::at(footEnum);
  }
};

template <>
struct MapEnumHelper<typename anymal_description::AnymalTopology::FootEnum, typename anymal_description::AnymalTopology::ContactEnum,
                     anymal_description::AnymalTopology> {
  static constexpr typename anymal_description::AnymalTopology::ContactEnum map(
      typename anymal_description::AnymalTopology::FootEnum footEnum) {
    return anymal_description::AnymalTopology::mapFootEnumToContactEnum::at(footEnum);
  }
};

} /* namespace internal */
} /* namespace romo */
