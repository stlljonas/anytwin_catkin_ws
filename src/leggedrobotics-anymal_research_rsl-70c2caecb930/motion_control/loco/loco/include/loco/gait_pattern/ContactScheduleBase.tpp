/*
 * ContactScheduleBase.cpp
 *
 *  Created on: April 18, 2018
 *      Author: Fabian Jenelten
 */

// loco
#include "loco/gait_pattern/ContactScheduleBase.hpp"

namespace loco {

template <typename LimbEnum>
ContactScheduleBase<LimbEnum>::ContactScheduleBase()
    : Base(),
      shouldBeGrounded_(true),
      timeUntilNextTouchDown_(-1.0),
      timeUntilNextLiftOff_(-1.0),
      timeSincePreviousTouchDown_(0.0),
      timeSincePreviousLiftOff_(0.0),
      status_(contact_schedule::Status::Undefined) {}

template <typename LimbEnum>
bool ContactScheduleBase<LimbEnum>::initialize(double /* dt */) {
  return setStanceGait();
}

template <typename LimbEnum>
unsigned int ContactScheduleBase<LimbEnum>::getNumberOfStanceLegs() const {
  return std::count(shouldBeGrounded_.begin(), shouldBeGrounded_.end(), true);
}

template <typename LimbEnum>
unsigned int ContactScheduleBase<LimbEnum>::getNumberOfSwingLegs() const {
  return std::count(shouldBeGrounded_.begin(), shouldBeGrounded_.end(), false);
}

template <typename LimbEnum>
bool ContactScheduleBase<LimbEnum>::shouldBeFullStancePhase() const {
  return (getNumberOfSwingLegs() == 0u);
}

template <typename LimbEnum>
bool ContactScheduleBase<LimbEnum>::shouldBeFullFlightPhase() const {
  return (getNumberOfStanceLegs() == 0u);
}

template <typename LimbEnum>
double ContactScheduleBase<LimbEnum>::getTimeLeftInStance(LimbEnum legId) const noexcept {
  if (shouldBeLegGrounded(legId)) {
    return timeUntilNextLiftOff_[legId];
  }
  return -1.0;
}

template <typename LimbEnum>
double ContactScheduleBase<LimbEnum>::getTimeLeftInSwing(LimbEnum legId) const noexcept {
  if (shouldBeLegSwing(legId)) {
    return timeUntilNextTouchDown_[legId];
  }
  return -1.0;
}

template <typename LimbEnum>
double ContactScheduleBase<LimbEnum>::getTimeSpentInStance(LimbEnum legId) const noexcept {
  if (shouldBeLegGrounded(legId)) {
    return timeSincePreviousTouchDown_[legId];
  }
  return -1.0;
}

template <typename LimbEnum>
double ContactScheduleBase<LimbEnum>::getTimeSpentInSwing(LimbEnum legId) const noexcept {
  if (shouldBeLegSwing(legId)) {
    return timeSincePreviousLiftOff_[legId];
  }
  return -1.0;
}

template <typename LimbEnum>
double ContactScheduleBase<LimbEnum>::getTimeUntilNextStance(LimbEnum legId) const noexcept {
  return timeUntilNextTouchDown_[legId];
}

template <typename LimbEnum>
double ContactScheduleBase<LimbEnum>::getTimeUntilNextSwing(LimbEnum legId) const noexcept {
  return timeUntilNextLiftOff_[legId];
}

template <typename LimbEnum>
bool ContactScheduleBase<LimbEnum>::shouldBeLegGrounded(LimbEnum legId) const noexcept {
  return shouldBeGrounded_[legId];
}

template <typename LimbEnum>
bool ContactScheduleBase<LimbEnum>::shouldBeLegSwing(LimbEnum legId) const noexcept {
  return !shouldBeGrounded_[legId];
}

template <typename LimbEnum>
bool ContactScheduleBase<LimbEnum>::setStanceGait() {
  status_ = contact_schedule::Status::Stand;
  std::fill(shouldBeGrounded_.begin(), shouldBeGrounded_.end(), true);
  std::fill(timeUntilNextTouchDown_.begin(), timeUntilNextTouchDown_.end(), -1.0);
  std::fill(timeUntilNextLiftOff_.begin(), timeUntilNextLiftOff_.end(), -1.0);
  std::fill(timeSincePreviousTouchDown_.begin(), timeSincePreviousTouchDown_.end(), 0.0);
  std::fill(timeSincePreviousLiftOff_.begin(), timeSincePreviousLiftOff_.end(), 0.0);
  return true;
}

template <typename LimbEnum>
contact_schedule::Status ContactScheduleBase<LimbEnum>::getStatus() const noexcept {
  return status_;
}

template <typename LimbEnum>
contact_schedule::EventHorizonStatus ContactScheduleBase<LimbEnum>::getEventHorizonStatus(LimbEnum legId) const noexcept {
  if (timeUntilNextLiftOff_[legId] >= 0.0 && timeUntilNextTouchDown_[legId] >= 0.0) {
    return contact_schedule::EventHorizonStatus::LiftOffAndTouchDown;
  } else if (timeUntilNextTouchDown_[legId] >= 0.0 && timeUntilNextLiftOff_[legId] < 0.0) {
    return contact_schedule::EventHorizonStatus::TouchDown;
  } else if (timeUntilNextTouchDown_[legId] < 0.0 && timeUntilNextLiftOff_[legId] >= 0.0) {
    return contact_schedule::EventHorizonStatus::LiftOff;
  } else {
    return contact_schedule::EventHorizonStatus::None;
  }
}

template <typename LimbEnum>
bool ContactScheduleBase<LimbEnum>::isGaitNonPeriodic() const noexcept {
  return (status_ == contact_schedule::Status::SwitchGait || status_ == contact_schedule::Status::SwitchToStand ||
          status_ == contact_schedule::Status::SwitchToWalk || status_ == contact_schedule::Status::ExecuteOneCycle);
}

template <typename LimbEnum>
bool ContactScheduleBase<LimbEnum>::isGaitPeriodic() const noexcept {
  return (status_ == contact_schedule::Status::Walk);
}

template <typename LimbEnum>
contact_schedule::Status ContactScheduleBase<LimbEnum>::nextStatus() const noexcept {
  switch (status_) {
    case contact_schedule::Status::SwitchGait: {
      return contact_schedule::Status::Walk;
    } break;
    case contact_schedule::Status::SwitchToStand: {
      return contact_schedule::Status::Stand;
    } break;
    case contact_schedule::Status::SwitchToWalk: {
      return contact_schedule::Status::Walk;
    } break;
    case contact_schedule::Status::ExecuteOneCycle: {
      return contact_schedule::Status::Stand;
    } break;
    default: { return status_; } break;
  }
}

}  // end namespace loco
