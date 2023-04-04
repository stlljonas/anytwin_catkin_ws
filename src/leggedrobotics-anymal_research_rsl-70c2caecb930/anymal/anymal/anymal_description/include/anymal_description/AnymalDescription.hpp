/*!
 * @file     AnymalDescription.hpp
 * @author   Dario Bellicoso
 * @date     Nov 6, 2017
 */

#pragma once

// anymal description
#include "anymal_description/AnymalDefinitions.hpp"
#include "anymal_description/AnymalTopology.hpp"
#include "anymal_description/LegEnum.hpp"
#include "anymal_description/robot_description_additions.hpp"

// romo
#include <romo/common/RobotDescription.hpp>

namespace anymal_description {

using ConcreteAnymalDescription = romo::ConcreteDescription<AnymalDefinitions, AnymalTopology>;
using AnymalDescription = romo::RobotDescription<ConcreteAnymalDescription>;

//! Shorthand for the number of legs of ANYmal
constexpr unsigned int numLegs = AnymalDescription::getNumLegs();

}  // namespace anymal_description
