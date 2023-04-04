/*
 * ContactScheduleBase.hpp
 *
 *  Created on: April 18, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// loco
#include "loco/gait_pattern/ContactScheduleLock.hpp"
#include "loco/gait_pattern/contact_schedule.hpp"

// stl
#include <vector>

class TiXmlHandle;

namespace loco {

template <typename LimbEnum>
class ContactScheduleBase : public ContactScheduleLock {
 public:
  using Base = ContactScheduleLock;

  ContactScheduleBase();
  virtual ~ContactScheduleBase() = default;

  /** Initializes the module.
   * @param dt  time step
   * @return true, iff successful
   */
  bool initialize(double dt) override;

  /*! @returns number of legs that should be in stance mode.
   */
  virtual unsigned int getNumberOfStanceLegs() const;

  /*! @returns number of legs that should be in swing mode.
   */
  virtual unsigned int getNumberOfSwingLegs() const;

  /*! @Returns true if all legs should be grounded swinging.
   */
  virtual bool shouldBeFullStancePhase() const;

  /*! @Returns true if all legs should be swinging.
   */
  virtual bool shouldBeFullFlightPhase() const;

  /*! @returns the time left in stance in seconds. If the leg is swinging, returns -1.
   * @param legId              index of leg
   */
  virtual double getTimeLeftInStance(LimbEnum legId) const noexcept;

  /*! @returns the time left in swing in seconds. If leg is stance, return -1.
   * @param legId              index of leg
   */
  virtual double getTimeLeftInSwing(LimbEnum legId) const noexcept;

  /*! @returns the time spent in stance in seconds. If the leg is swinging, returns -1.
   * @param legId              index of leg
   */
  virtual double getTimeSpentInStance(LimbEnum legId) const noexcept;

  /*! @returns the time spent in swing in seconds. If leg is stance, return -1.
   * @param legId              index of leg
   */
  virtual double getTimeSpentInSwing(LimbEnum legId) const noexcept;

  /*! @returns time until next stance phase starts in seconds.
   * @param legId              index of leg
   */
  virtual double getTimeUntilNextStance(LimbEnum legId) const noexcept;

  /*! @returns time until next swing phase starts in seconds.
   * @param legId              index of leg
   */
  virtual double getTimeUntilNextSwing(LimbEnum legId) const noexcept;

  /*! @returns true if leg should be grounded.
   * @param legId              index of leg
   */
  virtual bool shouldBeLegGrounded(LimbEnum legId) const noexcept;

  /*! @returns true if leg should be swinging.
   * @param legId              index of leg
   */
  virtual bool shouldBeLegSwing(LimbEnum legId) const noexcept;

  //! @Implements default gait while standing.
  virtual bool setStanceGait();

  //! @returns gait status.
  contact_schedule::Status getStatus() const noexcept;

  //! @returns horizon event status for leg.
  contact_schedule::EventHorizonStatus getEventHorizonStatus(LimbEnum legId) const noexcept;

  //! @True if the gait is in between a transition (for example stance -> walk or gait 1 -> gait 2).
  virtual bool isGaitNonPeriodic() const noexcept;

  //! @True if the gait is periodic.
  virtual bool isGaitPeriodic() const noexcept;

  //! @Status that would follow after transition has been completed.
  virtual contact_schedule::Status nextStatus() const noexcept;

  /*! @returns true if success.
   */
  virtual bool switchToStand(contact_schedule::ContactScheduleSwitchStatus& status) = 0;

  /*! @returns true if success.
   * @param isGaitForward         true if gait is started in forward motion.
   * @param startBalancingGait    true if the gait is started to stabilize the robot,
   *                              false, if the gait is started according to high-level commands.
   */
  virtual bool switchToWalk(contact_schedule::ContactScheduleSwitchStatus& status, bool isGaitForward, bool startBalancingGait = false) = 0;

 protected:
  //! Desired contact state (grounded/swing) for each leg.
  std_utils::EnumArray<LimbEnum, bool> shouldBeGrounded_;

  //! Time left until next stance event for each leg.
  std_utils::EnumArray<LimbEnum, double> timeUntilNextTouchDown_;

  //! Time left until next swing event for each leg.
  std_utils::EnumArray<LimbEnum, double> timeUntilNextLiftOff_;

  //! Time since previous stance event for each leg.
  std_utils::EnumArray<LimbEnum, double> timeSincePreviousTouchDown_;

  //! Time since previous swing event for each leg.
  std_utils::EnumArray<LimbEnum, double> timeSincePreviousLiftOff_;

  //! Status of the gait.
  contact_schedule::Status status_;
};

}  // namespace loco

#include <loco/gait_pattern/ContactScheduleBase.tpp>
