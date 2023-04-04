/*
 * GaitDescriptionLeg.hpp
 *
 *  Created on: April 18, 2018
 *      Author: Fabian Jenelten
 */

#pragma once


#include "vector"

// robot utils
#include "robot_utils/math/math.hpp"

namespace loco {
namespace contact_schedule {
namespace periodic {


class GaitDescriptionLeg {
public:
  GaitDescriptionLeg():
    liftOffPhaseEvents_(),
    touchDownPhaseEvents_()
  { }

  ~GaitDescriptionLeg() = default;

  GaitDescriptionLeg(const GaitDescriptionLeg&) = default;
  GaitDescriptionLeg& operator=(const GaitDescriptionLeg&) = default;
  GaitDescriptionLeg(GaitDescriptionLeg&&) = default;
  GaitDescriptionLeg& operator=(GaitDescriptionLeg&&) = default;

  void clear() noexcept {
    liftOffPhaseEvents_.clear();
    touchDownPhaseEvents_.clear();
  }

  double getPhaseUntilNextLiftOff(double stridePhase) const noexcept {
    return getPhaseUntilNextEvent(stridePhase, liftOffPhaseEvents_);
  }

  double getPhaseUntilNextTouchDown(double stridePhase) const noexcept {
    return getPhaseUntilNextEvent(stridePhase, touchDownPhaseEvents_);
  }

  const std::vector<double>& getLiftOffPhaseEvents() const noexcept {
    return liftOffPhaseEvents_;
  }

  const std::vector<double>& getTouchDownPhaseEvents() const noexcept  {
    return touchDownPhaseEvents_;
  }

  void setLiftOffPhaseEvents(const std::vector<double>& liftOffPhaseEvents) noexcept {
    liftOffPhaseEvents_ = liftOffPhaseEvents;
  }

  void setTouchDownPhaseEvents(const std::vector<double>& touchDownPhaseEvents) noexcept {
    touchDownPhaseEvents_ = touchDownPhaseEvents;
  }

  void push_back_phases(double liftOffPhase, double touchDownPhase) {
    liftOffPhaseEvents_.push_back(liftOffPhase);
    touchDownPhaseEvents_.push_back(touchDownPhase);
  }

  //! Get phase until next lift-off and touch-down event.
  void getPhaseUntilNextEvents(
      bool& shouldBeGrounded,
      double& phaseUntilNextLiftOff,
      double& phaseUntilNextTouchDown,
      double stridePhase) const noexcept {
    phaseUntilNextLiftOff   = getPhaseUntilNextLiftOff(stridePhase);
    phaseUntilNextTouchDown = getPhaseUntilNextTouchDown(stridePhase);
    shouldBeGrounded        = shouldBeLegGrounded(phaseUntilNextLiftOff, phaseUntilNextTouchDown);
    assert(phaseUntilNextTouchDown>=0.0 && phaseUntilNextTouchDown<=1.0 && phaseUntilNextLiftOff>=0.0 && phaseUntilNextLiftOff<=1.0);
  }

  /*
   * ! Get phase events in the unit interval [0, 1) s.t. stridePhase
   * becomes the new origin of the gait description.
   */
  void getNormalizedPhaseEvents(
      std::vector<double>& phaseUntilNextLiftOffEvents,
      std::vector<double>& phaseUntilNextTouchDownEvents,
      double stridePhase) const noexcept {
    phaseUntilNextLiftOffEvents   = liftOffPhaseEvents_;
    phaseUntilNextTouchDownEvents = touchDownPhaseEvents_;
    for (auto& phase : phaseUntilNextLiftOffEvents)   { robot_utils::mapToInterval(phase, stridePhase); }
    for (auto& phase : phaseUntilNextTouchDownEvents) { robot_utils::mapToInterval(phase, stridePhase); }
  }

  void getNormalizedAndSortedPhaseEvents(
      std::vector<double>& phaseUntilNextLiftOffEvents,
      std::vector<double>& phaseUntilNextTouchDownEvents,
      bool& shouldLegSwingAtGaitStart,
      double stridePhase) const {
    getNormalizedPhaseEvents(phaseUntilNextLiftOffEvents, phaseUntilNextTouchDownEvents, stridePhase);
    std::sort(phaseUntilNextLiftOffEvents.begin(), phaseUntilNextLiftOffEvents.end());
    std::sort(phaseUntilNextTouchDownEvents.begin(), phaseUntilNextTouchDownEvents.end());
    shouldLegSwingAtGaitStart = phaseUntilNextLiftOffEvents.back() > phaseUntilNextTouchDownEvents.back();
  }

  std::vector<double> getSortedLiftOffEvents() const {
    std::vector<double> liftOffPhaseEvents = liftOffPhaseEvents_;
    std::sort(liftOffPhaseEvents.begin(), liftOffPhaseEvents.end());
    return liftOffPhaseEvents;
  }

  std::vector<double> getSortedTouchDownEvents() const {
    std::vector<double> touchDownPhaseEvents = touchDownPhaseEvents_;
    std::sort(touchDownPhaseEvents.begin(), touchDownPhaseEvents.end());
    return touchDownPhaseEvents;
  }

private:
  double getPhaseUntilNextEvent(double stridePhase, double eventPhase) const noexcept {
    /*
     * Convention: If phase is at a lift-off event, the leg is considered to be swinging,
     * if a phase is at a touch-down event, the leg is considered to be grounded.
     */
    robot_utils::mapToInterval(eventPhase, stridePhase);
    if (eventPhase == 0.0) { eventPhase = 1.0; }
    return eventPhase;
  }

  double getPhaseUntilNextEvent(double stridePhase, const std::vector<double>& phaseEvents) const noexcept {
    double phaseUntilNextEvent = 1.0;
    for (const auto& event : phaseEvents) {
      phaseUntilNextEvent = std::fmin(phaseUntilNextEvent, getPhaseUntilNextEvent(stridePhase, event));
    }
    return phaseUntilNextEvent;
  }

  inline bool shouldBeLegGrounded(double phaseUntilNextLiftOff, double phaseLeftUntilTouchDown) const noexcept {
    return (phaseUntilNextLiftOff < phaseLeftUntilTouchDown);
  }

  //! Lift-off events defined in phase domain for a specific leg.
  std::vector<double> liftOffPhaseEvents_;

  //! Touch-down events defined in phase domain for a specific leg.
  std::vector<double> touchDownPhaseEvents_;
};

} // namespace contact_schedule
} // namespace periodic
} // namespace loco
