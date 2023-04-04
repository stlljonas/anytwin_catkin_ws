/*
 * SwingHeightPlanner.hpp
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

class SwingHeightPlanner : public SwingTrajectoryPlannerBase
{
 public:
  SwingHeightPlanner(const free_gait::AdapterBase& adapter, Parameters& parameters,
                     std::shared_ptr<ElevationMapUser> elevationMapUser);
  virtual ~SwingHeightPlanner();

  bool planSwingTrajectories(std::vector<free_gait::Step>& plan);
  bool planSwingHeight(free_gait::Footstep& footstep) const;
  bool checkForCollision(const free_gait::Footstep& footstep, const double margin, std::vector<std::pair<double, double>>& collisions) const;

 private:
  std::shared_ptr<ElevationMapUser> elevationMapUser_;
  const grid_map::SignedDistanceField* signedDistanceField_;
  free_gait::StepCompleter stepCompleter_;
  free_gait::StepParameters stepParameters_;
};

}
