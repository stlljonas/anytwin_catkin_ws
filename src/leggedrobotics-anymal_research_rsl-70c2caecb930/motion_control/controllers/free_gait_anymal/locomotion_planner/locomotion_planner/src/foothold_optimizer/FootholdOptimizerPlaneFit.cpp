/*
 * FootholdOptimizerPlaneFit.cpp
 *
 *  Created on: Mar 16, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */
#include "locomotion_planner/foothold_optimizer/FootholdOptimizerPlaneFit.hpp"
#include "locomotion_planner/common/type_defs.hpp"

// TODO Move.
#include <loco/terrain_perception/TerrainPerceptionFreePlane.hpp>

namespace locomotion_planner {

FootholdOptimizerPlaneFit::FootholdOptimizerPlaneFit(const free_gait::AdapterBase& adapter, Parameters& parameters,
                                                     PlanningData& planningData)
    : FootholdOptimizerBase(adapter, parameters, planningData)
{
}

FootholdOptimizerPlaneFit::~FootholdOptimizerPlaneFit()
{
}

bool FootholdOptimizerPlaneFit::optimizeFootholds(std::vector<free_gait::Step>& plan)
{
  loco::TerrainModelFreePlane terrain;
  std::vector<Position> footholds;
  for (const auto& limb : adapter_.getLimbs()) {
    if (adapter_.isLegGrounded(limb)) {
      footholds.push_back(adapter_.getPositionWorldToFootInWorldFrame(limb));
    }
  }
  loco::TerrainPerceptionFreePlane::generateTerrainModelFromPointsInWorldFrame(footholds, terrain);

  for (auto& step : plan) {
    if (step.hasLegMotion()) {
      for (auto& legMotion : step.getLegMotions()) {
        if (legMotion.second->getType() == free_gait::LegMotionBase::Type::Footstep) {
          auto& footstep = dynamic_cast<free_gait::Footstep&>(*legMotion.second);
          Position targetPosition = footstep.getTargetPosition();
          terrain.getHeight(targetPosition);
          Vector surfaceNormal;
          footstep.setTargetPosition(adapter_.getWorldFrameId(), targetPosition);
          if (terrain.getNormal(targetPosition, surfaceNormal)) {
            footstep.setSurfaceNormal(surfaceNormal);
          }
        }
      }
    }
  }

  return true;
}

} /* namespace locomotion_planner */
