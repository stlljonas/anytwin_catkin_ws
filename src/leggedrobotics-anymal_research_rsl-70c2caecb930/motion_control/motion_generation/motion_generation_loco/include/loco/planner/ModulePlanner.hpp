/*
 * ModulePlanner.hpp
 *
 *  Created on: Feb 5, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include <loco/planner/PlannerBoostThread.hpp>

// std utils
#include "std_utils/std_utils.hpp"

// boost
#include <boost/thread.hpp>

// message logger
#include "message_logger/message_logger.hpp"

// stl
#include <memory>

class TiXmlHandle;

namespace loco {

template<typename Module_, typename ModulePlan_>
class ModulePlanner: public PlannerBoostThread {
 public:
  using Module = Module_;
  using ModulePlan = ModulePlan_;

  explicit ModulePlanner(Module& module) :
    PlannerBoostThread(),
    module_(module),
    modulePlan_(),
    mutexModulePlan_(),
    chronoTimer_(),
    mutexComputationTime_(),
    lastComputationDuration_(0.0),
    elapsedTimeSinceLastSuccess_(0.0)
  { }

  ~ModulePlanner() override = default;

  bool loadParameters(const TiXmlHandle& hParameterSet) {
    return module_.loadParameters(hParameterSet);
  }

  bool addVariablesToLog(const std::string& ns) const {
    return module_.addVariablesToLog(ns);
  }

  bool addParametersToHandler(const std::string& ns) {
    return module_.addParametersToHandler(ns);
  }

  //! Initializes the planner.
  bool initialize(double dt, bool verbose = true) override {
    lastComputationDuration_ = 0.0;
    elapsedTimeSinceLastSuccess_ = 0.0;
    module_.initialize(dt);
    return PlannerBase::initialize(dt, verbose);
  }

  void setModulePlan(const ModulePlan& modulePlan) {
    boost::unique_lock<boost::shared_mutex> lock(mutexModulePlan_);
    modulePlan_ = modulePlan;
  }
  void getModulePlan(ModulePlan& modulePlan) {
    boost::shared_lock<boost::shared_mutex> lock(mutexModulePlan_);
    modulePlan = modulePlan_;
  }

  void resetLastComputationDuration() {
    boost::unique_lock<boost::shared_mutex> lock(mutexComputationTime_);
    lastComputationDuration_ = 0.0;
  }

  void resetElapesTimeSinceLastSuccess() {
    boost::unique_lock<boost::shared_mutex> lock(mutexComputationTime_);
    elapsedTimeSinceLastSuccess_ = 0.0;
  }

  bool plan() override {
    if (verbose_) {
      MELO_INFO_STREAM("[ModulePlanner::plan] Called plan.");
    }

    ModulePlan modulePlan;
    {
      boost::shared_lock<boost::shared_mutex> lock(mutexModulePlan_);
      modulePlan = modulePlan_;
    }

    if (verbose_) {
      MELO_INFO_STREAM("[ModulePlanner::plan] Computing motion plan.");
    }

    chronoTimer_.pinTime();
    module_.compute(modulePlan);
    {
      boost::unique_lock<boost::shared_mutex> lock(mutexComputationTime_);
      lastComputationDuration_ = chronoTimer_.getElapsedTimeSec();
      elapsedTimeSinceLastSuccess_ += lastComputationDuration_;
    }

    if (verbose_) {
      MELO_INFO_STREAM("[ModulePlanner::plan] Copying computed motion plan.");
    }

    {
      boost::unique_lock<boost::shared_mutex> lock(mutexModulePlan_);
      modulePlan_ = modulePlan;
    }

    return true;
  }

  bool runThread() override {
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
          MELO_WARN_STREAM("[ModulePlanner::runThread] Planner return false!");
        }
      }
      //--

      notifiedStartPlanning_ = false;
    }

    return true;
  }

  void updateControllerWithNewPlan() override {

  }

  double getLastComputationDuration() const {
    boost::shared_lock<boost::shared_mutex> lock(mutexComputationTime_);
    return lastComputationDuration_;
  }

  double getElapsedTimeSinceLastSuccess() const {
    boost::shared_lock<boost::shared_mutex> lock(mutexComputationTime_);
    return elapsedTimeSinceLastSuccess_;
  }

  void setElapsedTimeSinceLastSuccess(double elapsedTimeSinceLastSuccess) {
    boost::unique_lock<boost::shared_mutex> lock(mutexComputationTime_);
    elapsedTimeSinceLastSuccess_ = elapsedTimeSinceLastSuccess;
  }

  bool stopThread() override {
    lastComputationDuration_ = chronoTimer_.getElapsedTimeSec();
    MELO_INFO_STREAM("[ModulePlanner::stopThread] Stopping thread after " << lastComputationDuration_*1000.0 << " ms.");
    stopPlanningThread_ = true;
    chronoTimer_.pinTime();
    thread_.interrupt();
    module_.stop();
    if (thread_.joinable()) {
      thread_.join();
      lastComputationDuration_ += chronoTimer_.getElapsedTimeSec();
    }
    MELO_INFO_STREAM("[ModulePlanner::stopThread] Interruption time: " << chronoTimer_.getElapsedTimeSec()*1000.0  <<", total time: " << lastComputationDuration_*1000.0 << ".");
    return true;
  }

  double getExpectedPlanningTime() override {
    return 0.0;
  }

  double getTimeUntilNextPlanIsRequired() override {
    return 0.0;
  }

 protected:
  // optimizer.
  Module& module_;

  // plan.
  ModulePlan modulePlan_;
  boost::shared_mutex mutexModulePlan_;

  // timer.
  std_utils::HighResolutionClockTimer chronoTimer_;

  // timing.
  mutable boost::shared_mutex mutexComputationTime_;
  double lastComputationDuration_;
  double elapsedTimeSinceLastSuccess_;
};

} /* namespace loco */

