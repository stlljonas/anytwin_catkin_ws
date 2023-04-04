/*
 * FootholdOptimizerSameHeight.cpp
 *
 *  Created on: Oct 21, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */
#include <locomotion_planner/foothold_optimizer/FootholdOptimizerSameHeight.hpp>
#include "locomotion_planner/common/type_defs.hpp"

namespace locomotion_planner {

FootholdOptimizerSameHeight::FootholdOptimizerSameHeight(const free_gait::AdapterBase& adapter,
                                                               Parameters& parameters, PlanningData& planningData)
    : FootholdOptimizerBase(adapter, parameters, planningData)
{
}

FootholdOptimizerSameHeight::~FootholdOptimizerSameHeight()
{
}

bool FootholdOptimizerSameHeight::optimizeFootholds(std::vector<free_gait::Step>& plan)
{
  for (auto& step : plan) {
    if (step.hasLegMotion()) {
      for (auto& legMotion : step.getLegMotions()) {
        if (!adapter_.isLegGrounded(legMotion.first)) return true; // Ignore.
        if (legMotion.second->getType() == free_gait::LegMotionBase::Type::Footstep) {
          auto& footstep = dynamic_cast<free_gait::Footstep&>(*legMotion.second);
          Position targetPosition = footstep.getTargetPosition();
          targetPosition.z() = adapter_.getPositionWorldToFootInWorldFrame(legMotion.first).z();
          footstep.setTargetPosition(adapter_.getWorldFrameId(), targetPosition);
        }
      }
    }
  }

  return true;
}

} /* namespace locomotion_planner */
