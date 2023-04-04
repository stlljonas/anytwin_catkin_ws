/*
 * StepComputerMultiThreaded.hpp
 *
 *  Created on: Mar 16, 2016
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "anymal_ctrl_free_gait/base/StepComputerMultiThreaded.hpp"
#include "free_gait_core/step/Step.hpp"

namespace free_gait {

bool StepComputerMultiThreaded::initialize() {
  return PlannerBase::initialize(0, /* verbose = */ true);  // TODO(pfankhauser): Set dt.
}

bool StepComputerMultiThreaded::compute() {
  startPlanning();
  return true;
}

bool StepComputerMultiThreaded::isBusy() {
  return (hasStartedPlanning() && !hasFinishedPlanning());
}

bool StepComputerMultiThreaded::isDone() {
  return (hasStartedPlanning() && hasFinishedPlanning());
}

void StepComputerMultiThreaded::resetIsDone() {
  setHasStartedPlanning(false);
}

bool StepComputerMultiThreaded::plan() {
  Step step;
  boost::shared_lock<boost::shared_mutex> lock(mutex_);
  step = step_;
  lock.unlock();

  const bool result = step.compute();

  lock.lock();
  step_ = step;
  lock.unlock();

  return result;
}

double StepComputerMultiThreaded::getExpectedPlanningTime() {
  return 0.0;
}

double StepComputerMultiThreaded::getTimeUntilNextPlanIsRequired() {
  return 0.0;
}

void StepComputerMultiThreaded::updateControllerWithNewPlan() {}

}  // namespace free_gait
