/*
 * FootholdPlanner.hpp
 *
 *  Created on: Feb 5, 2016
 *      Author: Christian Gehring
 */

#pragma once

// loco
#include <loco/planner/PlannerBoostThread.hpp>

// std utils
#include "std_utils/timers/ChronoTimer.hpp"

// boost
#include <boost/thread.hpp>

// swing trajectory generation
#include "swing_trajectory_generation/SwingTrajectoryOptimizer.hpp"

class TiXmlHandle;

namespace loco {

class SwingTrajectoryPlanner: public PlannerBoostThread
{
 public:
  SwingTrajectoryPlanner();
  ~SwingTrajectoryPlanner() override = default;

  /*! Loads the parameters from the XML object
   * @param hParameterSet   handle
   * @return  true if all parameters could be loaded
   */
  virtual bool loadParameters(const TiXmlHandle& hParameterSet);

  //! Initializes the planner.
  virtual bool initialize(double dt, unsigned int numOfLegs, bool verbose = true);

  /*! The planning happens here.
   * @returns true if the planning was successful
   */
  bool plan() override;

  /*! This method is called whenever a new plan is available.
   */
  void updateControllerWithNewPlan() override;

  double getExpectedPlanningTime() override {
    return 0.0;
  }

  double getTimeUntilNextPlanIsRequired() override {
    return 0.0;
  }

  void setMotionPlan(const sto::MotionPlan& motionPlan, unsigned int legId);
  void getMotionPlans(std::vector<sto::MotionPlan>& motionPlan);

  const SwingTrajectoryOptimizer& getOptimizer() const;

  /*! This method should run in a different thread.
   * @returns true if successful
   */
  bool runThread() override ;

  double getLastComputationDuration() const;

  void resetLastComputationDuration();

  void resetOptimizedLegIndexes();
  bool isOptimizedLegIndex(unsigned int legId) const;

  bool stopThread() override;

  bool getDidOptimizationSuceeded() const;

 protected:
  //! The optimizer.
  std::unique_ptr<SwingTrajectoryOptimizer> optimizer_;

  //! Clock for measuring computation time.
  std_utils::HighResolutionClockTimer chronoTimer_;

  //! Motion plans (each leg has is own motion plan).
  std::vector<sto::MotionPlan> motionPlans_;

  //!
  boost::shared_mutex mutexMotionPlan_;
  mutable boost::shared_mutex mutexComputationTime_;

  //! Duration of the latest optimization
  double lastComputationDuration_;

  //! True if optimizations for all legs succeeded successfully.
  bool didOptimizationSuceeded_;

  //! True if leg index was optimized and if optimization was successfully.
  std::vector<bool> optimizedLegIndexes_;
};

} /* namespace loco */

