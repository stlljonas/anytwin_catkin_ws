/*
 * FootholdGeneratorOptimizedInvPend.hpp
 *
 *  Created on: Mar 15, 2018
 *      Author: Fabian Jenelten
 */

// loco
#include "loco/foothold_generation/FootholdGeneratorOptimizedInvPend.hpp"
#include "loco/foothold_generation/FootholdPlanInvPend.hpp"

// message logger
#include <message_logger/message_logger.hpp>

namespace loco {

FootholdGeneratorOptimizedInvPend::FootholdGeneratorOptimizedInvPend(WholeBody& wholeBody)
    : FootholdGeneratorOptimizedConstraint(wholeBody)
{
}



bool FootholdGeneratorOptimizedInvPend::setupCostFunction(unsigned int legId, const foothold_generator::FootholdPlan& plan) {
  // If we already have computed the objective.
  if (isCollisionConeConstraintActive_) { return true; }

  const auto* invPendPlan = dynamic_cast<const foothold_generator::FootholdPlanInvPend*>(&plan);
  if (invPendPlan == nullptr) {
    MELO_WARN_STREAM("[FootholdGeneratorOptimizedInvPend::setupCostFunction] Failed to cast motion plan.");
    return false;
  }

  // Approach to velocity projection of inverted pendulum model.
  const Position2d positionPlaneToFootHoldInPlaneFrame = invPendPlan->getVirtualPlaneFrame().getPosePlaneToWorld().inverseTransform(
      invPendPlan->getPositionWorldToVelocityProjectionInWorldFrame(legId)
  ).toImplementation().head<2>();
  addApproachingObjective(positionPlaneToFootHoldInPlaneFrame, invPendPlan->getWeightVelocityProjection());

  // Approach to previous solution.
  const Position2d positionPlaneToPreviousFootholdInPlaneFrame = plan.getVirtualPlaneFrame().getPosePlaneToWorld().inverseTransform(
      positionWorldToPreviousFootholdInWorldFrame_[legId]).toImplementation().head<2>();
  addApproachingObjective(positionPlaneToPreviousFootholdInPlaneFrame, invPendPlan->getWeightPreviousSolution());

  return true;
}

} /* namespace loco */
