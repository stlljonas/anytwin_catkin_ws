/*
 * ContactSchedulePeriodicSwitch.cpp
 *
 *  Created on: April 18, 2018
 *      Author: Fabian Jenelten
 */

// motion_generation
#include "motion_generation/ContactSchedulePeriodicSwitch.hpp"

// message logger
#include "message_logger/message_logger.hpp"

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>

namespace loco {
using namespace message_logger::color;

ContactSchedulePeriodicSwitch::ContactSchedulePeriodicSwitch(WholeBody& wholeBody)
    : Base(),
      torso_(*wholeBody.getTorsoPtr()),
      desiredGaitName_("undefined gait (set in ContactSchedulePeriodicSwitch::ContactSchedulePeriodicSwitch)"),
      desiredGaitId_(0u),
      activeGaitIdBeforeBalancing_(0u),
      legIdStartFirstGaitForwardDirection_(contact_schedule::LegEnumAnymal::SIZE),
      lastLegIdTouchDown_(),
      initStancePhase_(0.0),
      finalStancePhase_(0.5),
      eventContainerTransition_(),
      timeElapsedSinceGaitSwitchStart_(0.0),
      switchDuration_(0.0),
      activeGaitDuration_(0.0),
      isForwardDirection_(true)
{

}

bool ContactSchedulePeriodicSwitch::initialize(double dt) {
  if (!Base::initialize(dt)) { return false; }
  if(!updateDesiredGait(defaultGaitId_)) { return false; }
  if (contact_schedule::numOfLegs == 4u) {
    legIdStartFirstGaitForwardDirection_ = contact_schedule::LegEnumAnymal::RH;
  } else {
    legIdStartFirstGaitForwardDirection_ = *anymal_description::LegEnumIterator();
  }
  lastLegIdTouchDown_.clear();
  activeGaitIdBeforeBalancing_ = defaultGaitId_;
  isForwardDirection_ = true;
  return true;
}

bool ContactSchedulePeriodicSwitch::loadParameters(const TiXmlHandle& handle) {
  if(!Base::loadParameters(handle)) { return false; }

  TiXmlHandle limbCoordinationHandle = handle;
  if(!tinyxml_tools::getChildHandle(limbCoordinationHandle, handle, "LimbCoordination")) { return false; }
  TiXmlHandle contactScheduleHandle = handle;
  if(!tinyxml_tools::getChildHandle(contactScheduleHandle, limbCoordinationHandle, "ContactSchedule")) { return false; }
  const TiXmlHandle gaitPatternHandle = tinyxml_tools::getChildHandle(contactScheduleHandle, "GaitPattern");
  if(!tinyxml_tools::loadParameter(initStancePhase_, gaitPatternHandle, "init_stance_phase")) { return false; }
  return true;
}

bool ContactSchedulePeriodicSwitch::advance(double dt) {
  return Base::advance(dt);
}

bool ContactSchedulePeriodicSwitch::updateTimeEvents(double dt) {
  if (isGaitNonPeriodic()) {
    // Advance time.
    timeElapsedSinceGaitSwitchStart_ += dt;

    // Change gait.
    if (isDesiredGait() && desiredGaitId_!=activeGaitId_) {
      if (verbose_) {
        MELO_INFO_STREAM("[ContactSchedulePeriodicSwitch::updateTimeEvents] Reached end of active gait.");
      }
      if(!updateActiveGait(desiredGaitName_)) { return false; }
    }

    // Complete gait transition.
    if (isEndOfGaitTransition()) {
      if (verbose_) {
        MELO_INFO_STREAM("[ContactSchedulePeriodicSwitch::updateTimeEvents] Gait transition completed.");
      }

      eventContainerTransition_.clear();
      status_ = nextStatus();

      if (status_ == contact_schedule::Status::Stand) { stopGait(); }
      else if (status_ == contact_schedule::Status::Walk) { isLocked_ = false; }
      return Base::updateTimeEvents(dt);
    }

    // Update time until next event.
    const auto legEnumsUsedForWalk = getLegEnumsUsedForWalk();
    if (!legEnumsUsedForWalk.empty()) {
      for (const auto& legId : legEnumsUsedForWalk) {
        timeUntilNextTouchDown_[legId] = eventContainerTransition_.getTimeUntilNextTouchDownAtTime(legId, timeElapsedSinceGaitSwitchStart_); // ToDO: add stride feedback here!!
        timeUntilNextLiftOff_[legId]   = eventContainerTransition_.getTimeUntilNextLiftOffAtTime(legId, timeElapsedSinceGaitSwitchStart_);
        bool shouldLegBeGrounded       = true;

        switch(getEventHorizonStatus(legId)) {
          case contact_schedule::EventHorizonStatus::LiftOffAndTouchDown : {
            shouldLegBeGrounded = timeUntilNextLiftOff_[legId] < timeUntilNextTouchDown_[legId];
          } break;
          case contact_schedule::EventHorizonStatus::TouchDown : {
            shouldLegBeGrounded = false;
          } break;
          case contact_schedule::EventHorizonStatus::LiftOff : {
            shouldLegBeGrounded = true;
          } break;
          case contact_schedule::EventHorizonStatus::None : {
            shouldLegBeGrounded = true;
          } break;
        }

        if(!updatePreviousEvent(shouldLegBeGrounded, legId)) { return false; }
      }
    }
  }

  return Base::updateTimeEvents(dt);
}

unsigned int ContactSchedulePeriodicSwitch::getDesiredGaitId() const noexcept {
  return desiredGaitId_;
}

const std::string& ContactSchedulePeriodicSwitch::getDesiredGaitName() const noexcept {
  return desiredGaitName_;
}

bool ContactSchedulePeriodicSwitch::updateDesiredGait(unsigned int desiredGaitId) {
  if(!mapGaitIdToName(desiredGaitId, desiredGaitName_)) {
    MELO_WARN_STREAM("[ContactSchedulePeriodicSwitch::updateDesiredGait] Wrong gait index.");
    return false;
  }
  desiredGaitId_ = desiredGaitId;
  std::cout << blue << "[ContactSchedulePeriodicSwitch::updateDesiredGait]  Set new desired gait " << red << desiredGaitName_ << blue <<" ("<< desiredGaitId_ << ")"  << white << std::endl;
  return true;
}

bool ContactSchedulePeriodicSwitch::updateDesiredGait(const std::string& desiredGaitName) {
  desiredGaitName_ = desiredGaitName;
  desiredGaitId_   = mapGaitNameToId_[desiredGaitName];
  std::cout << blue << "[ContactSchedulePeriodicSwitch::updateDesiredGait] Set new desired gait " << red << desiredGaitName_ << blue <<" ("<< desiredGaitId_ << ")" << white << std::endl;
  return true;
}

bool ContactSchedulePeriodicSwitch::setDesiredGaitById(const unsigned int desiredGaitId, contact_schedule::ContactScheduleSwitchStatus& status) {
  status = contact_schedule::ContactScheduleSwitchStatus::Error;
  MELO_INFO_STREAM("[ContactSchedulePeriodicSwitch::setDesiredGaitById] Establish transition.");

  if (desiredGaitId == activeGaitId_) {
    MELO_INFO_STREAM("[ContactSchedulePeriodicSwitch::setDesiredGaitById] Gait " << activeGaitName_ << " is already active.");
    status = contact_schedule::ContactScheduleSwitchStatus::Switched;
    return true;
  }

  else if (status_ == contact_schedule::Status::Stand || status_ == contact_schedule::Status::SwitchToStand) {
    if(!updateDesiredGait(desiredGaitId)) { return false; }
    if(!updateActiveGait(desiredGaitId))  { return false; }
    status = contact_schedule::ContactScheduleSwitchStatus::Switched;
    return true;
  }

  else if (status_ != contact_schedule::Status::Walk && status_ != contact_schedule::Status::SwitchToWalk) {
    MELO_WARN_STREAM("[ContactSchedulePeriodicSwitch::setDesiredGaitById] Wrong status = " << contact_schedule::statusMap[status_] << ".");
    status = contact_schedule::ContactScheduleSwitchStatus::Error;
    return false;
  }

  if (!isValidGait(desiredGaitId)) {
    MELO_WARN_STREAM("[ContactSchedulePeriodicSwitch::setDesiredGaitById] Desired gait appears to be invalid.");
    status = contact_schedule::ContactScheduleSwitchStatus::NotFound;
    return false;
  }

  // Reset
  status_ = contact_schedule::Status::SwitchGait;
  timeElapsedSinceGaitSwitchStart_ = 0.0;
  eventContainerTransition_.clear();
  if(!updateDesiredGait(desiredGaitId)) { return false; }

  // Step 1: Go to stance using the active gait.
  double switchToStanceDuration;
  if(!computeEventContainerForSwitchToStand(switchToStanceDuration, false)){ return false; }

  // Step 2: Add full stance phase (empirical approach).
  const auto activeLegEnumsUsedForWalk = getActiveGaitDescription().getLegEnumsUsedForWalk();
  const auto desiredLegEnumsUsedForWalk = getDesiredGaitDescription().getLegEnumsUsedForWalk();
  const double activeStanceDuration  = getActiveGaitDescription().getEventContainer().findLargestFullStancePhase(activeLegEnumsUsedForWalk)*getActiveGaitDescription().getNominalStrideDuration();
  const double desiredStanceDuration = getDesiredGaitDescription().getEventContainer().findLargestFullStancePhase(desiredLegEnumsUsedForWalk)*getDesiredGaitDescription().getNominalStrideDuration();

  double stanceDuration = 0.0;
  if (activeStanceDuration>=0.0 && desiredStanceDuration>=0.0) {
    stanceDuration = std::fmax(activeStanceDuration, desiredStanceDuration);
  } else if (activeStanceDuration>=0.0 && desiredStanceDuration<0.0) {
    stanceDuration = activeStanceDuration;
  } else if (activeStanceDuration<0.0 && desiredStanceDuration>=0.0) {
    stanceDuration = desiredStanceDuration;
  } else {
    const double activeFlightDuration  = getActiveGaitDescription().getEventContainer().findLargestFullFlightPhase(activeLegEnumsUsedForWalk)*getActiveGaitDescription().getNominalStrideDuration();
    const double desiredFlightDuration = getDesiredGaitDescription().getEventContainer().findLargestFullFlightPhase(desiredLegEnumsUsedForWalk)*getDesiredGaitDescription().getNominalStrideDuration();

    if (activeFlightDuration>0.0 || desiredFlightDuration>0.0) {
      stanceDuration = std::fmax(activeFlightDuration, desiredFlightDuration);
    } else {
      stanceDuration = 0.1*getActiveGaitDescription().getNominalStrideDuration();
    }
  }
  activeGaitDuration_ = switchToStanceDuration+stanceDuration;

  // Step 3: Got to walk using the desired gait.
  if (!computeEventContainerForSwitchToWalk(switchDuration_, activeGaitDuration_, 2u, false)) { return false; }
  if (verbose_) { eventContainerTransition_.print(); }

  status = contact_schedule::ContactScheduleSwitchStatus::Switched;
  return true;
}

bool ContactSchedulePeriodicSwitch::switchToStand(contact_schedule::ContactScheduleSwitchStatus& status) {
  status = contact_schedule::ContactScheduleSwitchStatus::Error;
  MELO_INFO_STREAM("[ContactSchedulePeriodicSwitch::switchToStand] Establish transition.");

  if (status_ == contact_schedule::Status::Stand) {
    status = contact_schedule::ContactScheduleSwitchStatus::Switched;
    return true;
  }

  if (status_ != contact_schedule::Status::Walk && status_ != contact_schedule::Status::SwitchToWalk) {
    MELO_WARN_STREAM("[ContactSchedulePeriodicSwitch::switchToStand] Not walking, status = " << contact_schedule::statusMap[status_] << ".");
    status = contact_schedule::ContactScheduleSwitchStatus::Running;
    return false;
  }

  // Reset
  status_ = contact_schedule::Status::SwitchToStand;
  timeElapsedSinceGaitSwitchStart_ = 0.0;
  eventContainerTransition_.clear();

  /* Final stance phase: If we use the balancing gait, we want to be able to start it
   * asap again (for stability). Otherwise, we want to stay for a while in stance phase
   * to make sure the torso velocity settles to zero (before we can start the gait again).
   */
  const double finalStancePhase = (isBalancingGait() ? std::fmin(0.05, finalStancePhase_) : finalStancePhase_);

  // Establish transition.
  const bool closeCycle = (torso_.getMeasuredState().getLinearVelocityBaseInBaseFrame().norm() > 0.3);
  if(!computeEventContainerForSwitchToStand(switchDuration_, closeCycle)) {
    MELO_WARN_STREAM("[ContactSchedulePeriodicSwitch::switchToStand] ComputeEventContainerForSwitchToStand() returned false.");
    return false;
  }
  activeGaitDuration_ = switchDuration_;
  if(!addFinalStance(finalStancePhase)) {
    MELO_WARN_STREAM("[ContactSchedulePeriodicSwitch::switchToStand] addFinalStance() returned false.");
    return false;
  }
  status = contact_schedule::ContactScheduleSwitchStatus::Switched;

  return true;
}

bool ContactSchedulePeriodicSwitch::switchToWalk(contact_schedule::ContactScheduleSwitchStatus& status, bool isForwardDirection, bool startBalancingGait) {
  status = contact_schedule::ContactScheduleSwitchStatus::Error;
  MELO_INFO_STREAM("[ContactSchedulePeriodicSwitch::switchToWalk] Establish transition for " <<
      (startBalancingGait ? "balancing gait" : (isForwardDirection ? "forward motion" : "backward motion")) << ".");

  // Change leg indexes in case the direction of motion changes.
  const bool isDirectionChanging = (isForwardDirection != isForwardDirection_);
  isForwardDirection_ = isForwardDirection;
  if (isDirectionChanging) {
    for (auto& gait : gaitDescription_) {
      if (!gait.getIsReversibleGait()) {
        if (!gait.changeDirectionOfMotion()) {
          MELO_WARN_STREAM("[ContactSchedulePeriodicSwitch::switchToWalk] Failed to reorder leg indexes.");
          return false;
        }
      }
    }
  }

  if (status_ == contact_schedule::Status::Walk) {
    status = contact_schedule::ContactScheduleSwitchStatus::Switched;
    return true;
  }

  if (status_ != contact_schedule::Status::Stand) {
    MELO_WARN_STREAM("[ContactSchedulePeriodicSwitch::switchToWalk] Not in stance, status = " << contact_schedule::statusMap[status_] << ".");
    status = contact_schedule::ContactScheduleSwitchStatus::Running;
    return false;
  }

  if (!switchBetweenActiveAndBalancingGait(startBalancingGait)) { return false; }

  // Reset.
  status_ = contact_schedule::Status::SwitchToWalk;
  timeElapsedSinceGaitSwitchStart_ = 0.0;
  eventContainerTransition_.clear();
  std::fill(timeSincePreviousLiftOff_.begin(), timeSincePreviousLiftOff_.end(), 0.0);
  std::fill(timeSincePreviousTouchDown_.begin(), timeSincePreviousTouchDown_.end(), 0.0);


  /* Initial stance phase: If we start any gait we want to remain in stance to allow the torso
   * to reach a desirable pose before the first lift-off. For the balancing gait we don't want
   * to have any initial stance phase to ensure low reaction time (however, some small amount may be usefull
   * for the zmp optimizer).
   */
  const double initStancePhase = (isBalancingGait() ? std::fmin(0.05, initStancePhase_) : initStancePhase_);

  // Establish transition for two cycles.
  if (!computeEventContainerForSwitchToWalk(
      switchDuration_,
      initStancePhase*getActiveGaitDescription().getNominalStrideDuration(),
      2u, isDirectionChanging)) { return false; }
  if (verbose_) { eventContainerTransition_.print(); }
  status = contact_schedule::ContactScheduleSwitchStatus::Switched;
  return true;
}

bool ContactSchedulePeriodicSwitch::executeOneCycle(contact_schedule::ContactScheduleSwitchStatus& status, unsigned int numOfCycles) {
  MELO_INFO_STREAM("[ContactSchedulePeriodicSwitch::executeOneCycle] Establish transition.");
  status = contact_schedule::ContactScheduleSwitchStatus::Error;

  if (status_ != contact_schedule::Status::Stand) {
    MELO_WARN_STREAM("[ContactSchedulePeriodicSwitch::executeOneCycle] Not in stance, status = " << contact_schedule::statusMap[status_] << ".");
    status = contact_schedule::ContactScheduleSwitchStatus::Running;
    return false;
  }

  if (!switchBetweenActiveAndBalancingGait(false)) { return false; }

  // Reset
  status_ = contact_schedule::Status::ExecuteOneCycle;
  timeElapsedSinceGaitSwitchStart_ = 0.0;
  eventContainerTransition_.clear();
  isForwardDirection_ = true;
  std::fill(timeSincePreviousLiftOff_.begin(), timeSincePreviousLiftOff_.end(), 0.0);
  std::fill(timeSincePreviousTouchDown_.begin(), timeSincePreviousTouchDown_.end(), 0.0);

  // Establish transition for one cycles.
  if (!computeEventContainerForSwitchToWalk(
      switchDuration_,
      initStancePhase_*getActiveGaitDescription().getNominalStrideDuration(),
      numOfCycles, false)) { return false; }
  if(!addFinalStance(finalStancePhase_)) { return false; }
  if (verbose_) { eventContainerTransition_.print(); }

  status = contact_schedule::ContactScheduleSwitchStatus::Switched;
  return true;
}

bool ContactSchedulePeriodicSwitch::stopGait() {
  if (status_ == contact_schedule::Status::ForceStance) { return true; }

  if (status_ != contact_schedule::Status::Stand) {
    MELO_INFO_STREAM("[ContactScheduleZmp::stopGait] Gait is not in stance, status = " << contact_schedule::statusMap[status_] << ". Force shut down!");
    status_ = contact_schedule::Status::ForceStance;
  }

  isLocked_ = true;
  std::fill(shouldBeGrounded_.begin(), shouldBeGrounded_.end(), true);
  std::fill(timeUntilNextTouchDown_.begin(), timeUntilNextTouchDown_.end(), -1.0);
  std::fill(timeUntilNextLiftOff_.begin(), timeUntilNextLiftOff_.end(), -1.0);
  return true;
}

void ContactSchedulePeriodicSwitch::freeForceStance() {
  if(status_ == contact_schedule::Status::ForceStance) {
    status_ = contact_schedule::Status::Stand;
  }
}

bool ContactSchedulePeriodicSwitch::computeEventContainerForSwitchToWalk(
    double &switchDuration,
    double startTime,
    unsigned int numOfCycles,
    bool isDirectionChanging) {

  // Select gait.
  const bool isDesiredGait = (status_ == contact_schedule::Status::SwitchGait); // desired gait start with the first lift off.
  const auto& gaitDescription = (isDesiredGait ? getDesiredGaitDescription() : getActiveGaitDescription());
  const double strideDuration = gaitDescription.getNominalStrideDuration();

  // Find leg index to start the gait.
  if(!findStridePhaseToStartGait(isDirectionChanging, gaitDescription)) {
    MELO_WARN_STREAM("[ContactSchedulePeriodicSwitch::computeEventContainerForSwitchToWalk] Failed to find starting leg.");
    return false;
  }

  double largestTouchDownOverlappingPhase = 0.0;

  const auto& legEnumsUsedForWalk = gaitDescription.getLegEnumsUsedForWalk();
  if (!legEnumsUsedForWalk.empty()) {
    for (const auto& legId : legEnumsUsedForWalk) {
      // Get periodic gait description at desired stride phase.
      std::vector<double> phaseUntilNextLiftOffEvents;
      std::vector<double> phaseUntilNextTouchDownEvents;
      bool shouldLegSwingAtGaitStart;
      gaitDescription.getGaitDescriptionForLeg(legId).getNormalizedAndSortedPhaseEvents(
          phaseUntilNextLiftOffEvents, phaseUntilNextTouchDownEvents, shouldLegSwingAtGaitStart, stridePhase_);

      // Add lift-off events.
      for (const auto& phaseUntilNextLiftOffEvent : phaseUntilNextLiftOffEvents) {
        double liftOffTime = phaseUntilNextLiftOffEvent*strideDuration + startTime;
        for(unsigned int cycleId=0u; cycleId<numOfCycles; ++cycleId) {
          if(!eventContainerTransition_.addEvent(legId, contact_schedule::ContactEvent::LiftOff, liftOffTime, isDesiredGait)) { return false; }
          liftOffTime += strideDuration;
        }
      }

      // Add touch-down events.
      for (unsigned int id=0u; id<phaseUntilNextTouchDownEvents.size(); ++id) {
        double touchDownTime = phaseUntilNextTouchDownEvents[id]*strideDuration + startTime;
        if (id==0u && shouldLegSwingAtGaitStart) { touchDownTime += strideDuration; }
        for(unsigned int cycleId=0u; cycleId<numOfCycles; ++cycleId) {
          if(!eventContainerTransition_.addEvent(legId, contact_schedule::ContactEvent::TouchDown, touchDownTime, isDesiredGait)) { return false; }
          touchDownTime += strideDuration;
        }
      }

      // If leg should swing -> compute largest touch-down phase.
      if (phaseUntilNextLiftOffEvents.back() > phaseUntilNextTouchDownEvents.front()) {
        largestTouchDownOverlappingPhase = std::fmax(largestTouchDownOverlappingPhase, phaseUntilNextTouchDownEvents.front());
      }
    }
  }

  // Compute time duration where the switch will be active (i.e., the gait is not periodic).
  if (status_ == contact_schedule::Status::ExecuteOneCycle) {
    if (!eventContainerTransition_.getListOfEvents().empty()) {
      switchDuration = std::prev(eventContainerTransition_.getListOfEvents().end())->first;
    } else { return false; }
  } else {
    switchDuration = startTime + largestTouchDownOverlappingPhase*strideDuration;
    stridePhase_ += largestTouchDownOverlappingPhase;
  }

  return true;
}

bool ContactSchedulePeriodicSwitch::computeEventContainerForSwitchToStand(double &switchDuration, bool closeCycle) {
  switchDuration = 0.0;
  lastLegIdTouchDown_.clear();

  // Find threshold for swing duration (heuristic, safety for non-symmetric gaits).
  const double strideDuration = getActiveGaitDescription().getNominalStrideDuration();
  double liftOffTime; double touchDownTime;

  const auto legEnumsUsedForWalk = getLegEnumsUsedForWalk();
  if (!legEnumsUsedForWalk.empty()) {
    for (const auto& legId : legEnumsUsedForWalk) {
      // Get periodic gait description at desired stride phase.
      std::vector<double> phaseUntilNextLiftOffEvents;
      std::vector<double> phaseUntilNextTouchDownEvents;
      bool shouldLegSwingAtGaitStart;
      getActiveGaitDescription().getGaitDescriptionForLeg(legId).getNormalizedAndSortedPhaseEvents(
          phaseUntilNextLiftOffEvents, phaseUntilNextTouchDownEvents, shouldLegSwingAtGaitStart, stridePhase_);

      // If leg is grounded: Add next lift-off and touch-down event.
      if (!shouldLegSwingAtGaitStart) {
       liftOffTime = phaseUntilNextLiftOffEvents.front()*strideDuration;
        if(!eventContainerTransition_.addEvent(legId, contact_schedule::ContactEvent::LiftOff, liftOffTime)) { return false; }
        touchDownTime = phaseUntilNextTouchDownEvents.front()*strideDuration;
        if(!eventContainerTransition_.addEvent(legId, contact_schedule::ContactEvent::TouchDown, touchDownTime)) { return false; }
      }

      // If leg is swinging: Add touch-down (if closeCycle=true, add also proceeding lift-off and touch-down).
      else  {
        touchDownTime = phaseUntilNextTouchDownEvents.front()*strideDuration;
        if(!eventContainerTransition_.addEvent(legId, contact_schedule::ContactEvent::TouchDown, touchDownTime)) { return false; }

        if (closeCycle) {
          liftOffTime = phaseUntilNextLiftOffEvents.front()*strideDuration;
          if(!eventContainerTransition_.addEvent(legId, contact_schedule::ContactEvent::LiftOff, liftOffTime)) { return false; }
          touchDownTime += strideDuration;
          if(!eventContainerTransition_.addEvent(legId, contact_schedule::ContactEvent::TouchDown, touchDownTime)) { return false; }
        }
      }

      // Seek for the end of the transition.
      if (robot_utils::isLargerThan(touchDownTime, switchDuration)) {
        switchDuration = touchDownTime;
        lastLegIdTouchDown_.clear();
        lastLegIdTouchDown_.push_back(legId);
      } else if (robot_utils::areNear(touchDownTime, switchDuration)) {
        lastLegIdTouchDown_.push_back(legId);
      }
    }
  }

  return true;
}

bool ContactSchedulePeriodicSwitch::findStridePhaseToStartGait(
    bool isDirectionChanging,
    const contact_schedule::periodic::GaitDescription& gaitDescription) {

  // Find leg index to start the gait.
  auto legIdStartGait = (isForwardDirection_ ?
      legIdStartFirstGaitForwardDirection_ :
      contact_schedule::transformLegIdForDirectionChange(legIdStartFirstGaitForwardDirection_)
  );

  // Make sure leg index is used for walking.
  if (!gaitDescription.getLegEnumsUsedForWalk().empty()) {
    while (gaitDescription.isLegIdGrounded(legIdStartGait)) {
      legIdStartGait = contact_schedule::nextLeg(legIdStartGait);
    }
  } else {
    // In case no leg is used for walking.
    stridePhase_ = gaitDescription.getNominalStrideDuration();
    return true;
  }

  // In case we start the very first gait.
  if (lastLegIdTouchDown_.empty()) {
    stridePhase_ = gaitDescription.mapLegIndexToFirstEventPhase(legIdStartGait);
    return (stridePhase_!=-1.0);
  }

  if (!isDirectionChanging) {
    double phaseToStartGait;
    std::vector<contact_schedule::LegEnumAnymal> nextLifOffLegIndexes;

    for (const auto& legId : lastLegIdTouchDown_) {

      // Find leg indexes that will lift off next.
      if(!gaitDescription.getEventContainer().findNextLiftOffLegs(legId, nextLifOffLegIndexes, phaseToStartGait)) {
        // In case only one leg is used for walking.
        stridePhase_ = gaitDescription.mapLegIndexToFirstEventPhase(legIdStartGait);
        return (stridePhase_!=-1.0);
      }

      // Make sure non of these legs have already touch-down latest.
      for (const auto& nextLiftOffLegId : nextLifOffLegIndexes) {
        if (std::find(lastLegIdTouchDown_.begin(), lastLegIdTouchDown_.end(), nextLiftOffLegId)!=lastLegIdTouchDown_.end()) {
          nextLifOffLegIndexes.clear();
          break;
        }
      }

      if (!nextLifOffLegIndexes.empty()) {
        stridePhase_ = phaseToStartGait;
        return (stridePhase_!=-1.0);
      }
    }

    // In case we did not found leg index (we could take any leg).
    stridePhase_ = gaitDescription.mapLegIndexToFirstEventPhase(legIdStartGait);
    return (stridePhase_!=-1.0);
  }

  // Change direction of gait -> start with the leg that has touched down previously.
  stridePhase_ = gaitDescription.mapLegIndexToFirstEventPhase(lastLegIdTouchDown_.front());

  return (stridePhase_!=-1.0);
}

const contact_schedule::periodic::GaitDescription& ContactSchedulePeriodicSwitch::getDesiredGaitDescription() const {
  assert(activeGaitId_<gaitDescription_.size());
  return gaitDescription_[desiredGaitId_];
}

contact_schedule::periodic::GaitDescription& ContactSchedulePeriodicSwitch::getDesiredGaitDescription() {
  assert(activeGaitId_<gaitDescription_.size());
  return gaitDescription_[desiredGaitId_];
}

bool ContactSchedulePeriodicSwitch::isDesiredGait() const noexcept {
  if (status_ == contact_schedule::Status::SwitchGait) {
    return robot_utils::isEqualOrLargerThan(timeElapsedSinceGaitSwitchStart_, activeGaitDuration_);
  } return false;
}

bool ContactSchedulePeriodicSwitch::isEndOfGaitTransition() const noexcept {
  if (isGaitNonPeriodic()) {
    return robot_utils::isEqualOrLargerThan(timeElapsedSinceGaitSwitchStart_, switchDuration_);
  } return false;
}

bool ContactSchedulePeriodicSwitch::addFinalStance(double finalStancePhase) {
  if (eventContainerTransition_.getListOfEvents().empty()) {
    // In case no leg is used for walking (i.e., standing gait)
    return true;
  }
  if (finalStancePhase<=0.0) { return true; }

  // Add final stance phase.
  const double lastEventTime = std::prev(eventContainerTransition_.getListOfEvents().end())->first;
  const double finalStanceTime = finalStancePhase*getActiveGaitDescription().getNominalStrideDuration();
  if(!eventContainerTransition_.addEvent(
      contact_schedule::LegEnumAnymal::SIZE,
      contact_schedule::ContactEvent::EndOfGait,
      lastEventTime+finalStanceTime, false)
  ) {
    MELO_WARN_STREAM("[ContactSchedulePeriodicSwitch::addFinalStance] eventContainerTransition_.addEvent() returned false.");
    return false;
  }
  switchDuration_ += finalStanceTime;
  return true;
}

bool ContactSchedulePeriodicSwitch::isBalancingGait() {
  return (activeGaitId_ == mapGaitNameToId_[balancingGaitName_]);
}

bool ContactSchedulePeriodicSwitch::switchBetweenActiveAndBalancingGait(bool startBalancingGait) {
  // If gait is started to hold balance, use the balancing gait.
  if (startBalancingGait && !isBalancingGait()) {
    activeGaitIdBeforeBalancing_ = activeGaitId_;
    if(!updateDesiredGait(balancingGaitName_)) { return false; }
    if(!updateActiveGait(balancingGaitName_))  { return false; }
  }
  // If we previously started the gait for balancing -> switch back previous gait.
  else if (!startBalancingGait && isBalancingGait()) {
    activeGaitId_ = activeGaitIdBeforeBalancing_;
    if(!updateDesiredGait(activeGaitId_)) { return false; }
    if(!updateActiveGait(activeGaitId_))  { return false; }
  }
  return true;
}

double ContactSchedulePeriodicSwitch::getActiveGaitDuration() const noexcept {
  return activeGaitDuration_;
}

double ContactSchedulePeriodicSwitch::getSwitchDuration() const noexcept {
  return switchDuration_;
}

double ContactSchedulePeriodicSwitch::getTimeElapsedSinceGaitSwitchStart() const noexcept {
  return timeElapsedSinceGaitSwitchStart_;
}

bool ContactSchedulePeriodicSwitch::isForwardDirection() const noexcept {
  return isForwardDirection_;
}

bool ContactSchedulePeriodicSwitch::addVariablesToLog(const std::string& ns) const  {
  signal_logger::add(activeGaitDuration_, "activeGaitDuration", ns);
  signal_logger::add(finalStancePhase_, "finalStancePhase", ns);
  signal_logger::add(status_, "status", ns);
  signal_logger::add(switchDuration_, "switchDuration", ns);
  return true;
}

}  // end namespace loco
