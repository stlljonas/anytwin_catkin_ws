/*
 * FootstepPlannerBase.hpp
 *
 *  Created on: Mar 6, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include "locomotion_planner/common/type_defs.hpp"
#include "locomotion_planner/common/PlannerModuleBase.hpp"

namespace locomotion_planner {

class FootstepPlannerBase : public PlannerModuleBase
{
 public:
  FootstepPlannerBase(const free_gait::AdapterBase& adapter);
  virtual ~FootstepPlannerBase();

 protected:
  const free_gait::AdapterBase& adapter_;
};

} /* namespace locomotion_planner */
