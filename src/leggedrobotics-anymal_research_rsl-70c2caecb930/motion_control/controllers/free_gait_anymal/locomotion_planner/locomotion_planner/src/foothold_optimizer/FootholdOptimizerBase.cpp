/*
 * FootholdOptimizerBase.cpp
 *
 *  Created on: Mar 16, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */
#include "locomotion_planner/foothold_optimizer/FootholdOptimizerBase.hpp"

namespace locomotion_planner {

FootholdOptimizerBase::FootholdOptimizerBase(const free_gait::AdapterBase& adapter, Parameters& parameters,
                                             PlanningData& planningData)
    : adapter_(adapter),
      parameters_(parameters),
      planningData_(planningData)
{
}

FootholdOptimizerBase::~FootholdOptimizerBase()
{
}

} /* namespace */
