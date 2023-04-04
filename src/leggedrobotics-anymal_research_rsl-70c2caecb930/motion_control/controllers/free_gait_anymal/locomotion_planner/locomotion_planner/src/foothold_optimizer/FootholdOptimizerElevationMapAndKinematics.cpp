/*
 * FootholdOptimizerElevationMapAndKinematics.cpp
 *
 *  Created on: Mar 16, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */
#include "locomotion_planner/foothold_optimizer/FootholdOptimizerElevationMapAndKinematics.hpp"

#include <grid_map_core/grid_map_core.hpp>

#include <math.h>

namespace locomotion_planner {

FootholdOptimizerElevationMapAndKinematics::FootholdOptimizerElevationMapAndKinematics(
    const free_gait::AdapterBase& adapter, Parameters& parameters, PlanningData& planningData,
    std::shared_ptr<ElevationMapUser> elevationMapUser)
    : FootholdOptimizerBase(adapter, parameters, planningData),
      elevationMapUser_(elevationMapUser),
      footholdEmptyValidityType_({ }),
      footholdTerrainValidityType_({ PlanningData::FootholdValidityTypes::Terrain }),
      footholdKinematicsValidityType_({ PlanningData::FootholdValidityTypes::Kinematic }),
      footholdTerrainAndKinematicsValidityType_({ PlanningData::FootholdValidityTypes::Terrain, PlanningData::FootholdValidityTypes::Kinematic })
{
  state_.initialize(adapter_.getLimbs(), adapter_.getBranches());
  state_.setZero();
}

FootholdOptimizerElevationMapAndKinematics::~FootholdOptimizerElevationMapAndKinematics()
{
}

bool FootholdOptimizerElevationMapAndKinematics::optimizeFootholds(std::vector<free_gait::Step>& plan)
{
  if (!elevationMapUser_->isMapValid()) return false;

  // Update state.
  state_.setAllJointPositions(adapter_.getAllJointPositions());
  state_.setPositionWorldToBaseInWorldFrame(adapter_.getPositionWorldToBaseInWorldFrame());
  state_.setOrientationBaseToWorld(adapter_.getOrientationBaseToWorld());
  free_gait::StepQueue queue; // Empty.
  bool foundSolution = false;

  for (auto& step : plan) {
    if (step.hasLegMotion()) {
      for (auto& legMotion : step.getLegMotions()) {
        if (legMotion.second->getType() == free_gait::LegMotionBase::Type::Footstep) {
          auto& footstep = dynamic_cast<free_gait::Footstep&>(*legMotion.second);
          const Position nominalPositionInElevationMapFrame(
              adapter_.transformPosition(footstep.getFrameId(free_gait::ControlLevel::Position), elevationMapUser_->getMap().getFrameId(),
                                         footstep.getTargetPosition()));
          std::vector<Position> candidatePositions;

          Position position(nominalPositionInElevationMapFrame);
          Position optimizedPosition(nominalPositionInElevationMapFrame);
          
          // Check elevation map.
          bool success = elevationMapUser_->findClosestValidFoothold(position, candidatePositions);
          planningData_.addCandidateFootholds(legMotion.first, footholdEmptyValidityType_, candidatePositions);
          candidatePositions.clear();

          if (!success) {
            // No elevation map candidate found.
            MELO_WARN("No candidate footholds have been found in the elevation map");
          } else {
            // Found elevation map candidate.
            planningData_.addCandidateFoothold(legMotion.first, position, footholdTerrainValidityType_);

            // Check candidate for kinematics.
            if (!step.hasBaseMotion()) {
              MELO_ERROR("No base motion for this step?");
              return false;
            }
            footstep.setTargetPosition(elevationMapUser_->getMap().getFrameId(), position);
            if (step.getBaseMotion().prepareComputation(state_, step, queue, adapter_)) {
              // Found a candidate.
              planningData_.addCandidateFoothold(legMotion.first, position, footholdTerrainAndKinematicsValidityType_);
              optimizedPosition = position;
              foundSolution = true;
              // Transform optimized position back to world
              const Position optimizedPositionInWorld =
                adapter_.transformPosition(elevationMapUser_->getMap().getFrameId(), adapter_.getWorldFrameId(), optimizedPosition);
              footstep.setTargetPosition(adapter_.getWorldFrameId(), optimizedPositionInWorld);
              Vector surfaceNormal;
              if (elevationMapUser_->getSurfaceNormalForPosition(optimizedPosition, surfaceNormal)) {
                footstep.setSurfaceNormal(surfaceNormal);
              }
            }
          }
        }
      }
    }
  }

  return foundSolution;
}

} /* namespace locomotion_planner */
