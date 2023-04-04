/*
 * TorsoPlannerZmp.cpp
 *
 *  Created on: Feb 5, 2016
 *      Author: gech
 */

// loco
#include "loco/planner/TorsoPlannerZmp.hpp"

// message logger
#include "message_logger/message_logger.hpp"

namespace loco {
// ToDo: derive this class from generalized modele planner.

TorsoPlannerZmp::TorsoPlannerZmp() :
    PlannerBoostThread(),
    optimizer_(new ZmpOptimizerDynamicWalk()),
    chronoTimer_(),
    motionPlan_(),
    mutexMotionPlan_(),
    lastComputationDuration_(0.0),
    mutexComputationTime_(),
    didOptimizationSuceeded_(false),
    mutexDidOptimizationSucceed_(),
    terminationState_(zmp::TerminationState::Undefined),
    mutexTerminationState_()
{
}

bool TorsoPlannerZmp::loadParameters(const TiXmlHandle& hParameterSet) {
  return optimizer_->loadParameters(hParameterSet);
}

bool TorsoPlannerZmp::initialize(
    double dt,
    double wholeBodyMass,
    const Eigen::Matrix3d& torsoInertiaTensorInBaseFrame,
    bool verbose) {
  lastComputationDuration_ = 0.0;
  didOptimizationSuceeded_ = false;
  if(!optimizer_->initialize(wholeBodyMass, torsoInertiaTensorInBaseFrame)) { return false; }
  if(!PlannerBase::initialize(dt, verbose)) { return false; }
  return true;
}

void TorsoPlannerZmp::setMotionPlan(const zmp::MotionPlan& motionPlan) {
  boost::unique_lock<boost::shared_mutex> lock(mutexMotionPlan_);
  motionPlan_ = motionPlan;
}

void TorsoPlannerZmp::getMotionPlan(zmp::MotionPlan& motionPlan) {
  boost::shared_lock<boost::shared_mutex> lock(mutexMotionPlan_);
  motionPlan = motionPlan_;
}

void TorsoPlannerZmp::resetLastComputationDuration() {
  boost::unique_lock<boost::shared_mutex> lock(mutexComputationTime_);
  lastComputationDuration_ = 0.0;
}

bool TorsoPlannerZmp::plan() {
  if (verbose_) {
    MELO_INFO_STREAM("[TorsoPlannerZmp::plan] Called plan.");
  }

  chronoTimer_.pinTime();
  {
    boost::unique_lock<boost::shared_mutex> lock(mutexMotionPlan_);
    optimizer_->computeTrajectory(motionPlan_);

    {
      boost::unique_lock<boost::shared_mutex> lock(mutexTerminationState_);
      terminationState_ = motionPlan_.getTerminationState();
    }

    {
      boost::unique_lock<boost::shared_mutex> lock(mutexDidOptimizationSucceed_);
      didOptimizationSuceeded_ = motionPlan_.didOptimizationSucceeded();
    }

  }


  {
    boost::unique_lock<boost::shared_mutex> lock(mutexComputationTime_);
    lastComputationDuration_ = chronoTimer_.getElapsedTimeSec();
  }

  return didOptimizationSuceeded_;
}


bool TorsoPlannerZmp::runThread() {
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
        MELO_WARN_STREAM("[TorsoPlannerZmp::runThread] Planner return false!");
      }
    }
    //--

    notifiedStartPlanning_ = false;
  }

  return true;
}

void TorsoPlannerZmp::updateControllerWithNewPlan() {

}

const ZmpOptimizerBase& TorsoPlannerZmp::getZmpOptimizer() const {
  return *optimizer_;
}

double TorsoPlannerZmp::getLastComputationDuration() const {
  boost::shared_lock<boost::shared_mutex> lock(mutexComputationTime_);
  return lastComputationDuration_;
}

double TorsoPlannerZmp::getCurrentComputationDuration() const {
  return chronoTimer_.getElapsedTimeSec();
}

bool TorsoPlannerZmp::stopThread() {
  lastComputationDuration_ = chronoTimer_.getElapsedTimeSec();
  MELO_INFO_STREAM("[TorsoPlannerZmp::stopThread] Stopping thread after " << lastComputationDuration_*1000.0 << " ms.");
  stopPlanningThread_ = true;
  chronoTimer_.pinTime();
  thread_.interrupt();
  optimizer_->stop();
  if (thread_.joinable()) {
    thread_.join();
    lastComputationDuration_ += chronoTimer_.getElapsedTimeSec();
  }
  MELO_INFO_STREAM("[TorsoPlannerZmp::stopThread] Interruption time: " << chronoTimer_.getElapsedTimeSec()*1000.0  <<", total time: " << lastComputationDuration_*1000.0 << ".");
  return true;
}

bool TorsoPlannerZmp::getDidOptimizationSuceeded() const {
  boost::shared_lock<boost::shared_mutex> lock(mutexDidOptimizationSucceed_);
  return didOptimizationSuceeded_;
}

zmp::TerminationState TorsoPlannerZmp::getTerminationState() const {
  boost::shared_lock<boost::shared_mutex> lock(mutexTerminationState_);
  return terminationState_;
}

} /* namespace loco */
