/*
 * Geometry.hpp
 *
 *  Created on: Mar 14, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include "type_defs.hpp"

#include <free_gait_core/free_gait_core.hpp>

namespace locomotion_planner {

const Position2 getPlanarPositionFromPosition(const Position& position);
const Position getPositionFromPlanarPosition(const Position2& planarPosition);
const Pose getPoseFromPlanarPose(const PlanarPose& planarPose);
const PlanarPose getPlanarPoseFromPose(const Pose& pose);
const PlanarTwist getPlanarTwistFromTwist(const Twist& twist);
const Pose getFootprintPoseFromStance(const Stance& stance);
const Pose getNextFootprintPose(const free_gait::AdapterBase& adapter, const free_gait::StepQueue& plan);
const Position getFootPositionAfterActiveStep(const LimbEnum& limb, const free_gait::AdapterBase& adapter,
                                              const free_gait::StepQueue& plan);

template<class Type>
Type interpolateForSpeedFactor(const double speedFactor, const Type& valueSlow, const Type& valueFast)
{
  return valueSlow + speedFactor * (valueFast - valueSlow);
}

} /* namespace locomotion_planner */
