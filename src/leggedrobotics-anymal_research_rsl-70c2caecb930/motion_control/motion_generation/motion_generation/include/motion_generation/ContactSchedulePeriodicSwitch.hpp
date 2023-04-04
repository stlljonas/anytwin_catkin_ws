/*
 * ContactSchedulePeriodicSwitch.hpp
 *
 *  Created on: April 18, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// motion_generation
#include "motion_generation/ContactSchedulePeriodic.hpp"
#include "motion_generation/EventContainer.hpp"

// loco
#include "loco/common/WholeBody.hpp"

class TiXmlHandle;

namespace loco {

class ContactSchedulePeriodicSwitch : public ContactSchedulePeriodic {
public:
  using Base = ContactSchedulePeriodic;

  ContactSchedulePeriodicSwitch(WholeBody& wholeBody);
  ~ContactSchedulePeriodicSwitch() override = default;

  /** Initializes the module.
   * @param dt  time step
   * @return true, iff successful
   */
  bool initialize(double dt) override;

  /** Loads parameters from an xml file
   *  @param handle tinyxml handle
   *  @return true, iff successful
   */
  bool loadParameters(const TiXmlHandle& handle) override;

  /** Advances the module.
   * @param dt  time step
   * @return    true, iff successful
   */
  bool advance(double dt) override;

  //! Returns gait index of the desired gait.
  unsigned int getDesiredGaitId() const noexcept;

  //! Returns the name of the desired gait.
  const std::string& getDesiredGaitName() const noexcept;

  //! Set desired gait with gait index. Returns true if no error appears. didSwitched is true,
  // if the gait was set successfully and false, if the gait switcher was running.
  virtual bool setDesiredGaitById(const unsigned int desiredGaitId, contact_schedule::ContactScheduleSwitchStatus& status);

  //! Switch from walk to stand (soft stop).
  bool switchToStand(contact_schedule::ContactScheduleSwitchStatus& status) override;

  /*! @rSwitch from stand to walk.
   * @param isGaitForward         true if gait is started in forward motion.
   * @param startGaitForBalancing true if the gait is started to stabilize the robot,
   *                              false, if the gait is started according to high-level commands
   */
  bool switchToWalk(contact_schedule::ContactScheduleSwitchStatus& status, bool isGaitForward, bool startBalancingGait = false) override;

  //! Execute one gait cycle.
  bool executeOneCycle(contact_schedule::ContactScheduleSwitchStatus& status, unsigned int numOfCycles = 1u);

  //! Immediately switch to stand (hard stop).
  virtual bool stopGait();

  //! After the gait has been frozen, call this function to free the state.
  void freeForceStance();

  //! True if the gait switches and the used gait still corresponds to the active gait.
  bool isDesiredGait() const noexcept;

  //! True if the currently active gait is "balancing_gait".
  bool isBalancingGait();

  double getActiveGaitDuration() const noexcept;
  double getSwitchDuration() const noexcept;
  double getTimeElapsedSinceGaitSwitchStart() const noexcept;

  bool isForwardDirection() const noexcept;

  //! Add variables to the signal logger.
  bool addVariablesToLog(const std::string& ns = "/motion_generation/ContactSchedulePeriodicSwitch") const override;

protected:
  //! Update desired gait with gait index.
  bool updateDesiredGait(unsigned int activeGaitId);

  //! Update desired gait with gait name.
  bool updateDesiredGait(const std::string& desiredGaitName);

  /*! @Routine that updates timing vectors
   * @param dt  time step
   */
  bool updateTimeEvents(double dt) override;

  //! Establish transition stance to walk.
  bool computeEventContainerForSwitchToWalk(
      double &switchDuration,
      double startTime,
      unsigned int numOfCycles,
      bool isDirectionChanging);

  //! Establish transition walk to stance. Legs that are swinging go back to stance.
  // Stance legs will lift-off and touch-down. If closeCycle=true, swinging legs will lift-off again.
  bool computeEventContainerForSwitchToStand(double &switchDuration, bool closeCycle = false);

  //! Finds "optimal" leg index for starting the gait.
  bool findStridePhaseToStartGait(
      bool isDirectionChanging,
      const contact_schedule::periodic::GaitDescription& gaitDescription);

  //! Add a final stance phase to existing gait.
  bool addFinalStance(double finalStancePhase);

  //! Returns gait description corresponding to desired gait.
  const contact_schedule::periodic::GaitDescription& getDesiredGaitDescription() const;
  contact_schedule::periodic::GaitDescription& getDesiredGaitDescription();

  //! True if the end of a gait switch is reached.
  bool isEndOfGaitTransition() const noexcept;

  bool switchBetweenActiveAndBalancingGait(bool startBalancingGait);

  //! A reference to the torso.
  const TorsoBase& torso_;

  //! Desired gait name.
  std::string desiredGaitName_;

  //! Desired gait index.
  unsigned int desiredGaitId_;

  //! Previous active gait index used before the gait was started for balance the robot while standing.
  unsigned int activeGaitIdBeforeBalancing_;

  //! Leg index to start first gait (using forward gait).
  contact_schedule::LegEnumAnymal legIdStartFirstGaitForwardDirection_;

  //! After a switch to stance, the leg indexes that will have touched down latest will be stored in this vector.
  std::vector<contact_schedule::LegEnumAnymal> lastLegIdTouchDown_;

  //! Phase until the first leg lifts off during a switch to walk.
  double initStancePhase_;

  //! Phase that needs to pass after until gait can be switched to walk again.
  double finalStancePhase_;

  //! List of future events (important only during a gait transition).
  contact_schedule::EventContainer eventContainerTransition_;

  //! Timer that starts after a gait transition has started.
  double timeElapsedSinceGaitSwitchStart_;

  //! Duration of a switch.
  double switchDuration_;

  //! For gait switches: time the transition spends in the active gait.
  double activeGaitDuration_;

  //! True if the robot is walking forward.
  bool isForwardDirection_;
};

} // namespace loco
