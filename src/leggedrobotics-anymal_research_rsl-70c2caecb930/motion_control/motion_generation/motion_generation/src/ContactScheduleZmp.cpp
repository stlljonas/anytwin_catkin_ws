/*
 * ContactScheduleZmp.cpp
 *
 *  Created on: April 18, 2018
 *      Author: Fabian Jenelten
 */

// anymal_description
#include <anymal_description/LegEnum.hpp>

// motion_generation.
#include "motion_generation/ContactScheduleZmp.hpp"

// message logger.
#include "message_logger/message_logger.hpp"

// tinyxml tools.
#include <tinyxml_tools/tinyxml_tools.hpp>

namespace loco {
using namespace message_logger::color;

ContactScheduleZmp::ContactScheduleZmp(WholeBody& wholeBody)
    : Base(wholeBody),
      legs_(*wholeBody.getLegsPtr()),
      eventContainer_(),
      adaptedStrideDuration_(0.0),
      strideFeedback_(0.0),
      nominalStrideDuration_(0.0)
{

}

bool ContactScheduleZmp::advance(double dt) {
  // Compute number of regaining legs.
  auto numRegainingFeet = 0u;
  auto numSupportingLeg = 0u;
  for (const auto& legEnum : anymal_description::LegEnumIterator()) {
    auto* leg = legs_.getPtr(static_cast<unsigned int>(legEnum));
    if (leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactRecovery) {
      ++numRegainingFeet;
    } else if (leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support || leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant) {
      ++numSupportingLeg;
    }
  }

  /* Lock gait if one of the legs is regaining.
   * There should be at least two grounded contact for support the torso.
   * Otherwise, locking the gait will not improve anything.
   */
  if (numRegainingFeet>0u && numSupportingLeg>1u) {
    isLocked_ = true;
  } else {
    isLocked_ = false;
  }




  if (!Base::advance(dt)) { return false; }

  if(!updateListOfEvents(dt)) {
    MELO_WARN_STREAM("[ContactScheduleZmp::advance] Failed to update list of events.");
    return false;
  }

  if (!updateLegs()) {
    MELO_WARN_STREAM("[ContactScheduleZmp::advance] Failed to update torso and legs.");
    return false;
  }

  adaptedStrideDuration_ = getStrideDuration();
  strideFeedback_ = getActiveGaitDescription().getStrideFeedback();
  nominalStrideDuration_ = getActiveGaitDescription().getNominalStrideDuration();

  return true;
}

bool ContactScheduleZmp::updateListOfEvents(double dt) {
  eventContainer_.clear();
  std::vector<double> phaseUntilNextLiftOffEvents;
  std::vector<double> phaseUntilNextTouchDownEvents;

  // Increase first event duration if gait is locked.
  // (To make sure that the zmp optimizer does not reject the first support polygon as it may have a very small duration)
  double timeShift = 0.0;
  if (isLocked_) {
    const double firstEventTime = eventContainer_.getListOfEvents().begin()->first;
    const double expectedMinRegainDuration = 0.05*getNominalStrideDuration();
    timeShift = std::fmax(expectedMinRegainDuration-firstEventTime, 0.0);
  }

  if (isGaitPeriodic()) {
    const auto legEnumsUsedForWalk = getLegEnumsUsedForWalk();
    if (!legEnumsUsedForWalk.empty()) {
      for (const auto& legId : legEnumsUsedForWalk) {
        getActiveGaitDescription().getGaitDescriptionForLeg(legId).getNormalizedPhaseEvents(
            phaseUntilNextLiftOffEvents, phaseUntilNextTouchDownEvents, stridePhase_
        );

        for (const auto& liftOffPhase : phaseUntilNextLiftOffEvents) {
          if(!eventContainer_.addEvent(legId, contact_schedule::ContactEvent::LiftOff,
              liftOffPhase*getStrideDuration()+timeShift, false)) { return false; }
        }

        for (const auto& touchDownPhase : phaseUntilNextTouchDownEvents) {
          if(!eventContainer_.addEvent(legId, contact_schedule::ContactEvent::TouchDown,
              touchDownPhase*getStrideDuration()+timeShift, false)) { return false; }
        }
      }
    }
  }

  else if (isGaitNonPeriodic()) {
    bool success = true;
    switch(status_) {
      case contact_schedule::Status::SwitchToWalk :
      case contact_schedule::Status::ExecuteOneCycle :{
        const double timeWindow = (1.0 + initStancePhase_) * getActiveGaitDescription().getNominalStrideDuration();
        success &= eventContainerTransition_.extractListOfEventsInTimeWindow(
            eventContainer_,
            timeElapsedSinceGaitSwitchStart_,
            timeElapsedSinceGaitSwitchStart_+timeWindow);
      } break;

      case contact_schedule::Status::SwitchToStand :{
        success &= eventContainerTransition_.extractListOfEventsInTimeWindow(
            eventContainer_,
            timeElapsedSinceGaitSwitchStart_,
            timeElapsedSinceGaitSwitchStart_+switchDuration_);
      } break;

      case contact_schedule::Status::SwitchGait :{
        success &= eventContainerTransition_.extractListOfEventsFullCycle(
            eventContainer_,
            timeElapsedSinceGaitSwitchStart_);
      } break;

      default : break;
    }

    if (!success) {
      MELO_WARN_STREAM("[ContactScheduleZmp::updateListOfEvents] Failed to extract list of events");
      return false;
    }
  }

  if (verbose_) { eventContainer_.print(0.05); }

  return true;
}

const contact_schedule::ListOfEvents& ContactScheduleZmp::getListOfEvents() const noexcept {
  return eventContainer_.getListOfEvents();
}

bool ContactScheduleZmp::updateLegs() {
  for (const auto& legEnum : anymal_description::LegEnumIterator()) {
    auto* leg = legs_.getPtr(static_cast<unsigned int>(legEnum));

    // By default: Use previous stance or swing duration.
    double currentOrNextStanceDuration = leg->getContactSchedulePtr()->getStanceDuration();
    double currentOrNextSwingDuration  = leg->getContactSchedulePtr()->getSwingDuration();

    // By default: relative phases are -1.
    double currentSwingPhase = -1.0;
    double currentStancePhase = -1.0;

    // Note: The following computations are valid also for a non-periodic gait!
    switch(getEventHorizonStatus(legEnum)) {
      case contact_schedule::EventHorizonStatus::LiftOffAndTouchDown : {
        if (shouldBeLegGrounded(legEnum)) {
          currentOrNextStanceDuration = getTimeSpentInStance(legEnum) + getTimeLeftInStance(legEnum);
          currentOrNextSwingDuration  = getTimeUntilNextStance(legEnum) - getTimeUntilNextSwing(legEnum);
          if (currentOrNextStanceDuration>0.0) {
            currentStancePhase = getTimeSpentInStance(legEnum) / currentOrNextStanceDuration;
          }
        } else {
          currentOrNextStanceDuration = getTimeUntilNextSwing(legEnum) - getTimeUntilNextStance(legEnum);
          currentOrNextSwingDuration  = getTimeSpentInSwing(legEnum) + getTimeLeftInSwing(legEnum);
          if (currentOrNextSwingDuration>0.0) {
            currentSwingPhase  = getTimeSpentInSwing(legEnum) / currentOrNextSwingDuration;
          }
        }
      } break;
      case contact_schedule::EventHorizonStatus::TouchDown : {
        if (!shouldBeLegGrounded(legEnum)) {
          currentOrNextSwingDuration = getTimeSpentInSwing(legEnum) + getTimeLeftInSwing(legEnum);
          if (currentOrNextSwingDuration>0.0) {
            currentSwingPhase = getTimeSpentInSwing(legEnum) / currentOrNextSwingDuration;
          }
        }
      } break;
      case contact_schedule::EventHorizonStatus::LiftOff : {
        if (shouldBeLegGrounded(legEnum)) {
          currentOrNextStanceDuration = getTimeSpentInStance(legEnum) + getTimeLeftInStance(legEnum);
          if (currentOrNextStanceDuration>0.0) {
            currentStancePhase = getTimeSpentInStance(legEnum) / currentOrNextStanceDuration;
          }
        }
      } break;
    }

    leg->getContactSchedulePtr()->setPreviousSwingPhase(leg->getContactSchedule().getSwingPhase());
    leg->getContactSchedulePtr()->setPreviousStancePhase(leg->getContactSchedule().getStancePhase());

    leg->getContactSchedulePtr()->setShouldBeGrounded(shouldBeLegGrounded(legEnum));
    leg->getContactSchedulePtr()->setStanceDuration(currentOrNextStanceDuration);
    leg->getContactSchedulePtr()->setSwingDuration(currentOrNextSwingDuration);

    leg->getContactSchedulePtr()->setSwingPhase(currentSwingPhase);
    leg->getContactSchedulePtr()->setStancePhase(currentStancePhase);
  }

  return true;
}

bool ContactScheduleZmp::addVariablesToLog(const std::string & ns) const {
  signal_logger::add(adaptedStrideDuration_, "adaptedStrideDuration", ns);
  signal_logger::add(strideFeedback_, "strideFeedback", ns);
  signal_logger::add(nominalStrideDuration_, "nominalStrideDuration", ns);
  return ContactScheduleStrideControl::addVariablesToLog(ns);
}

}  // end namespace loco
