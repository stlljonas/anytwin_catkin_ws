/*
 * PlannerBase.cpp
 *
 *  Created on: Aug 27, 2015
 *      Author: Christian Gehring
 */

// loco
#include "loco/planner/PlannerBase.hpp"

// message logger
#include "message_logger/message_logger.hpp"

// stl
#include <chrono>

namespace loco {

PlannerBase::PlannerBase(bool isParallelized, int threadPriority)
    : isParallelized_(isParallelized),
      threadPriority_(threadPriority),
      stopPlanningThread_(false),
      hasFinishedPlanning_(true),
      hasStartedPlanning_(false),
      notifiedStartPlanning_(false),
      verbose_(true) {}

PlannerBase::~PlannerBase() {
  stopPlanningThread_ = true;
}

void PlannerBase::setPauseSimulationCallback(PauseSimulationCallback callback) {
  pauseSimulationCallback_ = callback;
}

void PlannerBase::pauseSimulation(bool pause) {
  if (!pauseSimulationCallback_.empty()) {
    pauseSimulationCallback_(pause);
  }
}

void PlannerBase::parallelize(bool isParallelized) {
  isParallelized_ = isParallelized;
}

bool PlannerBase::isParallelized() const {
  return isParallelized_;
}

bool PlannerBase::initialize(double dt, bool verbose) {
  MELO_INFO("[PlannerBase]  Start initialization.");

  stopPlanningThread_ = false;
  hasFinishedPlanning_ = true;
  hasStartedPlanning_ = false;
  notifiedStartPlanning_ = false;
  verbose_ = verbose;

  if (isParallelized_) {
    // Start worker thread
    if (!startThread()) {
      MELO_FATAL("[PlannerBase] Could not start worker!");
      return false;
    }
  }
  MELO_INFO("[PlannerBase]  End initialization.");
  return true;
}

bool PlannerBase::advance(double dt) {
  // MELO_INFO(" PlannerBase::advance begin")

  /* Get time left until stride is finished. */
  const double timeLeft = getTimeUntilNextPlanIsRequired();

  //-- Start planning if required.
  if (!hasStartedPlanning_) {
    // Planning has not yet started, check if we should start now.
    const double expectedPlanningTime = isParallelized_ ? getExpectedPlanningTime() : dt + 0.000001;
    if (timeLeft < expectedPlanningTime) {
      // if we don't start now, we get into troubles
      MELO_INFO("New motion plan needs to be optimized.");
      startPlanning();
    }
  }
  //--

  //-- Update controller with new motion plan if required.
  if (timeLeft <= dt) {
    // New motion plan is required now because the stride is over.
    if (!hasFinishedPlanning_) {
      // Planning is late! Oh oh. What shall we do?
      MELO_FATAL("New plan is required, but planning has not finished yet!");
    } else {
      // New motion plan was planned and we can update the controller now.
      MELO_INFO("Update controller with new plan.");
      updateControllerWithNewPlan();
      // Indicate that we can re-plan for the next stride.
      hasStartedPlanning_ = false;
    }
  }
  //--

  // MELO_INFO(" PlannerBase::advance end")
  return true;
}

void PlannerBase::startPlanning() {
  hasStartedPlanning_ = true;
  hasFinishedPlanning_ = false;
  if (isParallelized_) {
    notifiedStartPlanning_ = true;
    cvStartPlanning_.notify_all();
  } else {
    pauseSimulation(true);
    runPlan();
    pauseSimulation(false);
  }
}

bool PlannerBase::stopPlanning() {
  MELO_INFO("[PlannerBase] Stopping ... ");
  if (isParallelized_) {
    stopPlanningThread_ = true;
    notifiedStartPlanning_ = true;
    cvStartPlanning_.notify_all();
    // cancel thread
    stopThread();
  }
  MELO_INFO("[PlannerBase] ... stopped.");
  return true;
}

bool PlannerBase::runThread() {
  while (!stopPlanningThread_) {
    /* Suspend the thread until start planning is notified.
     * We use the conditional variable for that, but we need the flag notifiedStartPlanning_
     * to handle spurious wake-ups.
     */
    boost::unique_lock<boost::mutex> lock(mutexStartPlanning_);
    cvStartPlanning_.wait(lock, [this]() { return (notifiedStartPlanning_ || stopPlanningThread_); });
    if (stopPlanningThread_) {
      return true;
    }

    //-- plan
    if (!runPlan()) {
      return false;
    }
    //--

    notifiedStartPlanning_ = false;
  }

  return true;
}

bool PlannerBase::runPlan() {
  if (verbose_) {
    MELO_INFO_THROTTLE(0.0, "Start planning...");
  }
  hasFinishedPlanning_ = false;

  // start time measurement
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  int64_t elapsedTimeNSecs;
  start = std::chrono::steady_clock::now();
  //--

  //--
  // Plan here
  //--
  bool result = plan();

  //-- measure time
  end = std::chrono::steady_clock::now();
  elapsedTimeNSecs = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  //--

  if (verbose_) {
    MELO_INFO_THROTTLE(0.0, "... finished planning after %lf ms.", (double)elapsedTimeNSecs * 1e-6);
  }

  hasFinishedPlanning_ = true;

  return result;
}

bool PlannerBase::hasFinishedPlanning() {
  return hasFinishedPlanning_;
}

bool PlannerBase::hasStartedPlanning() {
  return hasStartedPlanning_;
}

void PlannerBase::setHasStartedPlanning(bool hasStarted) {
  hasStartedPlanning_ = hasStarted;
}

}  // namespace loco
