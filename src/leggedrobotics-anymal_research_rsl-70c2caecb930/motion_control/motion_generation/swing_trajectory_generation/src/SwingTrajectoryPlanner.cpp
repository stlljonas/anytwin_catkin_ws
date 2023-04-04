/*
 * SwingTrajectoryPlanner.cpp
 *
 *  Created on: Feb 5, 2016
 *      Author: gech
 */

// loco
#include "swing_trajectory_generation/SwingTrajectoryPlanner.hpp"

// message logger
#include "message_logger/message_logger.hpp"

namespace loco {

SwingTrajectoryPlanner::SwingTrajectoryPlanner() :
    PlannerBoostThread(),
    optimizer_(new SwingTrajectoryOptimizer()),
    chronoTimer_(),
    motionPlans_(),
    mutexMotionPlan_(),
    mutexComputationTime_(),
    lastComputationDuration_(0.0),
    didOptimizationSuceeded_(false),
    optimizedLegIndexes_()
{

}

bool SwingTrajectoryPlanner::loadParameters(const TiXmlHandle& hParameterSet) {
  return optimizer_->loadParameters(hParameterSet);
}

bool SwingTrajectoryPlanner::initialize(double dt, unsigned int numOfLegs, bool verbose) {
  lastComputationDuration_ = 0.0;
  didOptimizationSuceeded_ = false;
  optimizedLegIndexes_.resize(numOfLegs);
  std::fill(optimizedLegIndexes_.begin(), optimizedLegIndexes_.end(), false);
  motionPlans_.resize(numOfLegs);
  optimizer_->initialize();
  return PlannerBase::initialize(dt, verbose);
}

void SwingTrajectoryPlanner::setMotionPlan(const sto::MotionPlan& motionPlan, unsigned int legId) {
  boost::unique_lock<boost::shared_mutex> lock(mutexMotionPlan_);
  optimizedLegIndexes_[legId] = true;
  motionPlans_[legId] = motionPlan;
}

void SwingTrajectoryPlanner::getMotionPlans(std::vector<sto::MotionPlan>& motionPlan) {
  boost::shared_lock<boost::shared_mutex> lock(mutexMotionPlan_);
  for (unsigned int legId=0u; legId<motionPlans_.size(); ++legId) {
    // Only copy the motion plan if we have optimized for it and the if the optimization was successful.
    if (optimizedLegIndexes_[legId]) {
      motionPlan[legId] = motionPlans_[legId];
    }
  }
}

void SwingTrajectoryPlanner::resetLastComputationDuration() {
  boost::unique_lock<boost::shared_mutex> lock(mutexComputationTime_);
  lastComputationDuration_ = 0.0;
}

void SwingTrajectoryPlanner::resetOptimizedLegIndexes() {
  boost::shared_lock<boost::shared_mutex> lock(mutexComputationTime_);
  std::fill(optimizedLegIndexes_.begin(), optimizedLegIndexes_.end(), false);
}

bool SwingTrajectoryPlanner::isOptimizedLegIndex(unsigned int legId) const {
  boost::shared_lock<boost::shared_mutex> lock(mutexComputationTime_);
  return optimizedLegIndexes_[legId];
}

bool SwingTrajectoryPlanner::plan() {
  if (verbose_) {
    MELO_INFO_STREAM("[SwingTrajectoryPlanner::plan] Called plan.");
  }

  chronoTimer_.pinTime();
  unsigned int numOfSucceededOptimizations = 0u;

  {
    boost::unique_lock<boost::shared_mutex> lock(mutexMotionPlan_);

    for (unsigned int legId=0u; legId<motionPlans_.size(); ++legId) {
      if (optimizedLegIndexes_[legId]) {

        if (verbose_) {
          MELO_INFO_STREAM("[SwingTrajectoryPlanner::plan] Computing motion plan for leg " << legId << ".");
        }

        // Run optimization.
        if (!optimizer_->computeTrajectory(motionPlans_[legId])) {
          optimizedLegIndexes_[legId] = false;
          MELO_WARN_STREAM("[SwingTrajectoryPlanner::plan] Optimization failed for " << legId << ".");
        } else { ++numOfSucceededOptimizations; }
      }
    }
  }

  // Compute computation duration.
  {
    boost::unique_lock<boost::shared_mutex> lock(mutexComputationTime_);
    lastComputationDuration_ = chronoTimer_.getElapsedTimeSec();
  }




  // We consider the optimization to be successful if at least one trajectory could be generated.
  didOptimizationSuceeded_ = (numOfSucceededOptimizations>0u);

  return didOptimizationSuceeded_;
}


bool SwingTrajectoryPlanner::runThread() {
  while(!stopPlanningThread_)
  {
    /* Suspend the thread until start planning is notified.
     * We use the conditional variable for that, but we need the flag notifiedStartPlanning_
     * to handle spurious wake-ups.
     */
    boost::unique_lock<boost::mutex> lock(mutexStartPlanning_);
    cvStartPlanning_.wait(lock,
      [this] () {
        return (notifiedStartPlanning_ || stopPlanningThread_);
      }
    );
    if (stopPlanningThread_) {
      return true;
    }

    //-- plan
    if(!runPlan()) {
//      return false;
      if (verbose_) {
        MELO_WARN_STREAM("[SwingTrajectoryPlanner::runThread] Planner return false!");
      }
    }
    //--

    notifiedStartPlanning_ = false;
  }

  return true;
}

void SwingTrajectoryPlanner::updateControllerWithNewPlan() {

}

const SwingTrajectoryOptimizer& SwingTrajectoryPlanner::getOptimizer() const {
  return *optimizer_;
}

double SwingTrajectoryPlanner::getLastComputationDuration() const {
  boost::shared_lock<boost::shared_mutex> lock(mutexComputationTime_);
  return lastComputationDuration_;
}


bool SwingTrajectoryPlanner::stopThread() {
  if (verbose_) {
    MELO_INFO_STREAM("[SwingTrajectoryPlanner::stopThread] Stopping thread.");
  }
  stopPlanningThread_ = true;
  thread_.interrupt();
  return true;
}

bool SwingTrajectoryPlanner::getDidOptimizationSuceeded() const {
  return didOptimizationSuceeded_;
}

} /* namespace loco */
