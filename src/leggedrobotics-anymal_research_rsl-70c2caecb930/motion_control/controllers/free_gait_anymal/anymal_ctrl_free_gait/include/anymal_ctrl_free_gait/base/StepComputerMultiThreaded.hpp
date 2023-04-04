/*
 * StepComputerMultiThreaded.hpp
 *
 *  Created on: Mar 16, 2016
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "free_gait_core/step/StepComputer.hpp"

// Loco
#include <loco/planner/PlannerBoostThread.hpp>

// Boost
#include <boost/thread/shared_mutex.hpp>

namespace free_gait {

class StepComputerMultiThreaded : public StepComputer, private loco::PlannerBoostThread {
 public:
  StepComputerMultiThreaded() : StepComputer() { parallelize(true); }
  ~StepComputerMultiThreaded() override = default;
  bool initialize() override;
  bool compute() override;
  bool isBusy() override;
  bool isDone() override;
  void resetIsDone() override;

  bool plan() override;
  double getExpectedPlanningTime() override;
  double getTimeUntilNextPlanIsRequired() override;
  void updateControllerWithNewPlan() override;

 private:
  boost::shared_mutex mutex_;
};

}  // namespace free_gait
