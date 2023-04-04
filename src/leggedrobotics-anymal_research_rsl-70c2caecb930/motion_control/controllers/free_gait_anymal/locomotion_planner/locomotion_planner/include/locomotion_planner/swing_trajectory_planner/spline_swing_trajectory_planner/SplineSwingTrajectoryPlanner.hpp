/*
 * SplineSwingTrajectoryPlanner.hpp
 *
 *  Created on: Sep 7, 2017
 *      Author: Péter Fankhauser
 */

#pragma once

#include "locomotion_planner/swing_trajectory_planner/SwingTrajectoryPlannerBase.hpp"
#include "locomotion_planner/common/ElevationMapUser.hpp"

#include <free_gait_core/step/StepCompleter.hpp>
#include <free_gait_core/step/StepParameters.hpp>

namespace locomotion_planner {

class SplineSwingTrajectoryPlanner : public SwingTrajectoryPlannerBase
{
 public:
  SplineSwingTrajectoryPlanner(const free_gait::AdapterBase& adapter, Parameters& parameters,
                               std::shared_ptr<ElevationMapUser> elevationMapUser);
  virtual ~SplineSwingTrajectoryPlanner();

  bool planSwingTrajectories(std::vector<free_gait::Step>& plan);
  bool planSwingTrajectory(const free_gait::Footstep& footstep,
                           free_gait::EndEffectorTrajectory& endEffectorTrajectory);

 private:
  std::shared_ptr<ElevationMapUser> elevationMapUser_;
  free_gait::StepCompleter stepCompleter_;
  free_gait::StepParameters stepParameters_;
};

}
