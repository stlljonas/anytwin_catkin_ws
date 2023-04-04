/*
 * FootstepPlannerBase.cpp
 *
 *  Created on: Mar 6, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "locomotion_planner/footstep_planner/FootstepPlannerBase.hpp"

namespace locomotion_planner {

FootstepPlannerBase::FootstepPlannerBase(const free_gait::AdapterBase& adapter)
    : adapter_(adapter)
{
}

FootstepPlannerBase::~FootstepPlannerBase()
{
}

} /* namespace */
