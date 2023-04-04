/*
 * SwingTrajectoryPlannerBase.hpp
 *
 *  Created on: Sep 7, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include "locomotion_planner/common/type_defs.hpp"
#include "locomotion_planner/common/Parameters.hpp"

namespace locomotion_planner {

class SwingTrajectoryPlannerBase
{
 public:
  SwingTrajectoryPlannerBase(const free_gait::AdapterBase& adapter, Parameters& parameters);
  virtual ~SwingTrajectoryPlannerBase();

  virtual bool planSwingTrajectories(std::vector<free_gait::Step>& plan) = 0;

 protected:
  const free_gait::AdapterBase& adapter_;
  Parameters& parameters_;
};

} /* namespace locomotion_planner */
