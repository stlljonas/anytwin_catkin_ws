/*
 * TorsoPlannerZmp.hpp
 *
 *  Created on: Feb 5, 2016
 *      Author: Christian Gehring
 */

#pragma once

// loco
#include "loco/planner/PlannerBoostThread.hpp"
#include "loco/common/WholeBody.hpp"

// zmp optimzer
#include "zmp_optimizer/zmp_optimizer.hpp"
#include "zmp_optimizer/ZmpOptimizerDynamicWalk.hpp"

// std utils
#include "std_utils/timers/ChronoTimer.hpp"

// boost
#include "boost/thread.hpp"

class TiXmlHandle;

namespace loco {

class TorsoPlannerZmp: public PlannerBoostThread
{
 public:
  TorsoPlannerZmp();
  ~TorsoPlannerZmp() override = default;


  /*! Loads the parameters from the XML object
   * @param hParameterSet   handle
   * @return  true if all parameters could be loaded
   */
  virtual bool loadParameters(const TiXmlHandle& hParameterSet);

  //! Initializes the planner.
  virtual bool initialize(
      double dt,
      double wholeBodyMass,
      const Eigen::Matrix3d& torsoInertiaTensorInBaseFrame,
      bool verbose = true);

  /*! The planning happens here.
   * @returns true if the planning was successful
   */
  virtual bool plan();

  /*! This method is called whenever a new plan is available.
   */
  virtual void updateControllerWithNewPlan();

  double getExpectedPlanningTime() {
    return 0.0;
  }

  double getTimeUntilNextPlanIsRequired() {
    return 0.0;
  }

  void setMotionPlan(const zmp::MotionPlan& motionPlan);
  void getMotionPlan(zmp::MotionPlan& motionPlan);

  const ZmpOptimizerBase& getZmpOptimizer() const;

  /*! This method should run in a different thread.
   * @returns true if successful
   */
  virtual bool runThread();

  double getLastComputationDuration() const;

  double getCurrentComputationDuration() const;

  void resetLastComputationDuration();

  bool stopThread();

  bool getDidOptimizationSuceeded() const;
  zmp::TerminationState getTerminationState() const;

 protected:
  //! Optimizer.
  std::unique_ptr<ZmpOptimizerDynamicWalk> optimizer_;

  //! Clock.
  std_utils::HighResolutionClockTimer chronoTimer_;

  //! Motion plan.
  zmp::MotionPlan motionPlan_;
  boost::shared_mutex mutexMotionPlan_;

  //! Computation duration.
  mutable boost::shared_mutex mutexComputationTime_;
  double lastComputationDuration_;

  //! True if optimization was a success.
  bool didOptimizationSuceeded_;
  mutable boost::shared_mutex mutexDidOptimizationSucceed_;

  //! Termination state of the ooptimization.
  zmp::TerminationState terminationState_;
  mutable boost::shared_mutex mutexTerminationState_;
};

} /* namespace loco */

