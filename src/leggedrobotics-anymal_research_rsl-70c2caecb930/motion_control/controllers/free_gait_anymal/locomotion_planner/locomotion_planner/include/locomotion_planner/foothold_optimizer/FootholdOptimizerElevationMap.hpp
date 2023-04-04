/*
 * FootholdOptimizer.hpp
 *
 *  Created on: Mar 16, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include "locomotion_planner/foothold_optimizer/FootholdOptimizerBase.hpp"
#include "locomotion_planner/common/ElevationMapUser.hpp"
#include "locomotion_planner/common/type_defs.hpp"

#include <free_gait_core/free_gait_core.hpp>

namespace locomotion_planner {

class FootholdOptimizerElevationMap : public FootholdOptimizerBase
{
 public:
  FootholdOptimizerElevationMap(const free_gait::AdapterBase& adapter, Parameters& parameters,
                                PlanningData& planningData, std::shared_ptr<ElevationMapUser> elevationMapUser);
  virtual ~FootholdOptimizerElevationMap();

  bool optimizeFootholds(std::vector<free_gait::Step>& plan);

 private:
  std::shared_ptr<ElevationMapUser> elevationMapUser_;
  const std::vector<PlanningData::FootholdValidityTypes> footholdEmptyValidityType_;
  const std::vector<PlanningData::FootholdValidityTypes> footholdTerrainValidityType_;
};

} /* namespace locomotion_planner */
