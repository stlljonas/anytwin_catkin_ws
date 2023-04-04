/*
 * SwingTrajectoryPlannerBase.cpp
 *
 *  Created on: Sep 7, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "locomotion_planner/swing_trajectory_planner/SwingTrajectoryPlannerBase.hpp"

namespace locomotion_planner {

SwingTrajectoryPlannerBase::SwingTrajectoryPlannerBase(const free_gait::AdapterBase& adapter, Parameters& parameters)
    : adapter_(adapter),
      parameters_(parameters)
{
}

SwingTrajectoryPlannerBase::~SwingTrajectoryPlannerBase()
{
}

} /* namespace */
