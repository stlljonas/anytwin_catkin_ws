/*
 * PlannerBase.hpp
 *
 *  Created on: Aug 27, 2015
 *      Author: Christian Gehring
 */

#pragma once

// boost
#include <boost/thread.hpp>

namespace loco {

using PauseSimulationCallback = boost::function<void(bool pause)>;

class PlannerBase {
 public:
  explicit PlannerBase(bool isParallelized = true, int threadPriority = 0);
  virtual ~PlannerBase();

  void parallelize(bool isParallelized);
  bool isParallelized() const;

  /*! Initializes the planner.
   *
   */
  virtual bool initialize(double dt, bool verbose);

  /*! This method is invoked every control update.
   * Checks if it should re-plan and if a new plan is
   * available.
   */
  virtual bool advance(double dt);

  /*!
   * @returns the expected time needed for planning.
   */
  virtual double getExpectedPlanningTime() = 0;
  /*!
   * @returns the time left until the plan is required
   * by the controller.
   */
  virtual double getTimeUntilNextPlanIsRequired() = 0;

  /*!  Call simulation to pause/resume.
   *  If the planning is not parallized, the simulation
   *  will be paused during the planning.
   *  @param pause    pause or resume
   */
  void pauseSimulation(bool pause);

  /*! Sets the callback function to pause the simulation.
   * @param callback
   */
  void setPauseSimulationCallback(PauseSimulationCallback callback);

  bool hasFinishedPlanning();
  bool hasStartedPlanning();
  void setHasStartedPlanning(bool hasStarted);

  /*! Start the planning process.
   */
  virtual void startPlanning();

  /*! Stop the planning process.
   */
  virtual bool stopPlanning();

 protected:
  /*! The planning happens here.
   * @returns true if the planning was successful
   */
  virtual bool plan() = 0;

  /*! This method is called whenever a new plan is available.
   */
  virtual void updateControllerWithNewPlan() = 0;

  virtual bool startThread() = 0;
  virtual bool stopThread() = 0;

  /*! This method should run in a different thread.
   * @returns true if successful
   */
  virtual bool runThread();

  /*! Wrapper of plan to measure computation time.
   *
   */
  virtual bool runPlan();

 protected:
  //! if true, the planning is done in a thread
  bool isParallelized_;

  int threadPriority_;

  //! Callback that will pause the simulation.
  PauseSimulationCallback pauseSimulationCallback_;

  boost::atomic<bool> stopPlanningThread_;
  boost::atomic<bool> hasFinishedPlanning_;
  boost::atomic<bool> hasStartedPlanning_;
  boost::atomic<bool> notifiedStartPlanning_;
  boost::condition_variable_any cvStartPlanning_;
  boost::mutex mutexStartPlanning_;
  bool verbose_;
};

} /* namespace loco */
