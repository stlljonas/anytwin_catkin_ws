/*
 * FootholdOptimizerSameHeight.hpp
 *
 *  Created on: Oct 21, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include "locomotion_planner/foothold_optimizer/FootholdOptimizerBase.hpp"

#include <free_gait_core/free_gait_core.hpp>

namespace locomotion_planner {

class FootholdOptimizerSameHeight : public FootholdOptimizerBase
{
 public:
  FootholdOptimizerSameHeight(const free_gait::AdapterBase& adapter, Parameters& parameters, PlanningData& planningData);
  virtual ~FootholdOptimizerSameHeight();

  bool optimizeFootholds(std::vector<free_gait::Step>& plan);
};

} /* namespace locomotion_planner */
