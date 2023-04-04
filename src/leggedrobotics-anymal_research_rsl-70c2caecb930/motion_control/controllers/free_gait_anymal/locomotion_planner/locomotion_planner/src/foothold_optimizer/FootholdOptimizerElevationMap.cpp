/*
 * FootholdOptimizerElevationMap.cpp
 *
 *  Created on: Mar 16, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */
#include "locomotion_planner/foothold_optimizer/FootholdOptimizerElevationMap.hpp"

#include <grid_map_core/grid_map_core.hpp>

#include <math.h>

namespace locomotion_planner {

FootholdOptimizerElevationMap::FootholdOptimizerElevationMap(const free_gait::AdapterBase& adapter,
                                                             Parameters& parameters,
                                                             PlanningData& planningData,
                                                             std::shared_ptr<ElevationMapUser> elevationMapUser)
    : FootholdOptimizerBase(adapter, parameters, planningData),
      elevationMapUser_(elevationMapUser),
      footholdEmptyValidityType_({ }),
      footholdTerrainValidityType_({ PlanningData::FootholdValidityTypes::Terrain })
{
}

FootholdOptimizerElevationMap::~FootholdOptimizerElevationMap()
{
}

bool FootholdOptimizerElevationMap::optimizeFootholds(std::vector<free_gait::Step>& plan)
{
  if (!elevationMapUser_->isMapValid()) return false;

  for (auto& step : plan) {
    if (step.hasLegMotion()) {
      for (auto& legMotion : step.getLegMotions()) {
        if (legMotion.second->getType() == free_gait::LegMotionBase::Type::Footstep) {
          auto& footstep = dynamic_cast<free_gait::Footstep&>(*legMotion.second);
          Position position(footstep.getTargetPosition());
          std::vector<Position> candidatePositions;
          bool success = elevationMapUser_->findClosestValidFoothold(position, candidatePositions);
          planningData_.addCandidateFootholds(legMotion.first, footholdEmptyValidityType_, candidatePositions);
          if (!success) return false;
          planningData_.addCandidateFoothold(legMotion.first, position, footholdTerrainValidityType_);
          footstep.setTargetPosition(adapter_.getWorldFrameId(), position);
          Vector surfaceNormal;
          if (elevationMapUser_->getSurfaceNormalForPosition(position, surfaceNormal)) {
            footstep.setSurfaceNormal(surfaceNormal);
          }
        }
      }
    }
  }

  return true;
}

} /* namespace locomotion_planner */
