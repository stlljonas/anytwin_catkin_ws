/*
 * FootholdOptimizerBase.hpp
 *
 *  Created on: Mar 16, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include "locomotion_planner/common/Parameters.hpp"
#include "locomotion_planner/common/PlanningData.hpp"

#include <free_gait_core/free_gait_core.hpp>

namespace locomotion_planner {

class FootholdOptimizerBase
{
 public:
  FootholdOptimizerBase(const free_gait::AdapterBase& adapter, Parameters& parameters, PlanningData& planningData);
  virtual ~FootholdOptimizerBase();

  virtual bool optimizeFootholds(std::vector<free_gait::Step>& plan) = 0;

 protected:
  const free_gait::AdapterBase& adapter_;
  Parameters& parameters_;
  PlanningData& planningData_;
};

} /* namespace locomotion_planner */
