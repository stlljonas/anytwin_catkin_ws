/*!
 * @file     AnymalDescription.cpp
 * @author   Dario Bellicoso
 * @date     Nov 6, 2017
 */

// anymal description
#include "anymal_description/AnymalDescription.hpp"

// std_utils
#include <std_utils/std_utils.hpp>

namespace anymal_description {

constexpr std_utils::KeyArray<AnymalTopology::LimbEnum> AnymalTopology::limbKeys;
constexpr std_utils::KeyArray<AnymalTopology::BranchEnum> AnymalTopology::branchKeys;
constexpr std_utils::KeyArray<AnymalTopology::JointNodeEnum> AnymalTopology::jointNodeKeys;
constexpr std_utils::KeyArray<AnymalTopology::JointEnum> AnymalTopology::jointKeys;
constexpr std_utils::KeyArray<AnymalTopology::ActuatorNodeEnum> AnymalTopology::actuatorNodeKeys;
constexpr std_utils::KeyArray<AnymalTopology::ActuatorEnum> AnymalTopology::actuatorKeys;
constexpr std_utils::KeyArray<AnymalTopology::BodyNodeEnum> AnymalTopology::bodyNodeKeys;
constexpr std_utils::KeyArray<AnymalTopology::BodyEnum> AnymalTopology::bodyKeys;
constexpr std_utils::KeyArray<AnymalTopology::GeneralizedCoordinatesEnum> AnymalTopology::generalizedCoordinateKeys;
constexpr std_utils::KeyArray<AnymalTopology::GeneralizedVelocitiesEnum> AnymalTopology::generalizedVelocityKeys;
constexpr std_utils::KeyArray<AnymalTopology::ContactEnum> AnymalTopology::contactKeys;
constexpr std_utils::KeyArray<AnymalTopology::ContactStateEnum> AnymalTopology::contactStateKeys;
constexpr std_utils::KeyArray<AnymalTopology::CoordinateFrameEnum> AnymalTopology::coordinateFrameKeys;
constexpr std_utils::KeyArray<AnymalTopology::FrameTransformEnum> AnymalTopology::frameTransformKeys;
constexpr std_utils::KeyArray<AnymalTopology::FootEnum> AnymalTopology::footKeys;

}  // namespace anymal_description
