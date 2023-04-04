/*
 * TerrainModelFreeGait.cpp
 *
 *  Created on: Feb 17, 2016
 *      Author: P. Fankhauser
 */

#include "anymal_ctrl_free_gait/base/TerrainModelFreeGait.hpp"

// STD
#include <limits>

namespace loco {

bool TerrainModelFreeGait::getNormal(const AD::LimbEnum limb, loco::Vector& normalInWorldFrame) const {
  if (executor_.getState().hasSurfaceNormal(limb)) {
    normalInWorldFrame = executor_.getState().getSurfaceNormal(limb);
  } else {
    normalInWorldFrame = defaultFootNormalInWorldFrame_;
  }
  return true;
}

bool TerrainModelFreeGait::getNormal(const loco::Position& positionInWorldFrame, loco::Vector& normalInWorldFrame) const {
  free_gait::LimbEnum candidateLimb;
  double candidateDistance = std::numeric_limits<double>::infinity();
  for (const auto& limb : executor_.getAdapter().getLimbs()) {
    const Position footPositionOfLimb(executor_.getAdapter().getPositionWorldToFootInWorldFrame(limb));
    double distanceForLimb = (footPositionOfLimb - positionInWorldFrame).norm();
    if (distanceForLimb < candidateDistance) {
      candidateDistance = distanceForLimb;
      candidateLimb = limb;
    }
  }

  if (candidateDistance < 0.03) {
    getNormal(candidateLimb, normalInWorldFrame);
  } else {
    normalInWorldFrame = normalInWorldFrame_;
  }

  return true;
}

bool TerrainModelFreeGait::getFrictionCoefficientForFoot(const loco::Position& positionInWorldFrame, double& frictionCoefficient) const {
  frictionCoefficient = frictionCoefficientBetweenTerrainAndFoot_;
  return true;
}

}  // namespace loco
