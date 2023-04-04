/*
 * foothold_generator.hpp
 *
 *  Created on: Mar 4, 2017
 *      Author: Dario Bellicoso
 */

// std_utils
#include <std_utils/std_utils.hpp>

// Eigen
#include <Eigen/Core>

#pragma once

namespace loco {
namespace foothold_generator {

enum IneqConstraints {
  KinematicFeasibility = 0,
  CollisionAvoidance,
  SIZE_OF_INEQCONSTRAINTS
};

/*
 * Nominal Foothold Generation Technique:
 *    > InvertedPendulum            Foothold generation based on inverted pendulum model and flight controller feedback.
 *    > InvertedPendulumMotionGen   Similar to InvertedPendulum but with some adjustments (e.g. footholds do not move in odom if robot is not disturbed)
 */
CONSECUTIVE_ENUM(NominalFootholdGeneration, InvertedPendulum, InvertedPendulumMotionGen, Undefined)

/*
 * Foothold Generation Optimizer: Find optimal foothold in the vicinity of the nominal foothold.
 *    > BlindQP                     A QP is formulated that imposes constraints on kinematics. No vision included.
 */
CONSECUTIVE_ENUM(FootholdGenerationOptimizer, BlindQP, Undefined)

//! Foothold center used to generate footholds.
CONSECUTIVE_ENUM(FootholdCenter, Base, TorsoCom, WholeBodyCom)

} /* namespace foothold_generator */
} /* namespace loco */
