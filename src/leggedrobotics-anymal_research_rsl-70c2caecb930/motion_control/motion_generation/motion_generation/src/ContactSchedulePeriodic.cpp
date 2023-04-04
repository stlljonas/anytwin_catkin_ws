/*
 * ContactSchedulePeriodic.cpp
 *
 *  Created on: April 18, 2018
 *      Author: Fabian Jenelten
 */

// anymal_description
#include <anymal_description/LegEnum.hpp>

// loco
#include "motion_generation/ContactSchedulePeriodic.hpp"

// message logger
#include "message_logger/message_logger.hpp"

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>

namespace loco {
using namespace message_logger::color;

ContactSchedulePeriodic::ContactSchedulePeriodic()
    : Base(),
      stridePhase_(-1.0),
      gaitDescription_(),
      activeGaitName_("undefined gait (set in ContactSchedulePeriodic::ContactSchedulePeriodic)"),
      activeGaitId_(0u),
      mapGaitNameToId_(),
      defaultGaitId_(0u),
      verbose_(false),
      balancingGaitName_("balancing_gait") {
}


bool ContactSchedulePeriodic::initialize(double dt) {
  if (!Base::initialize(dt)) { return false; }
  if(!updateActiveGait(defaultGaitId_)) { return false; }
  stridePhase_ = 0.0;
  isLocked_    = true;
  return true;
}

bool ContactSchedulePeriodic::loadParameters(const TiXmlHandle& handle) {
  MELO_DEBUG_STREAM(magenta << "[ContactSchedulePeriodic] " << blue << "Load parameters." << def)

  // Reset.
  gaitDescription_.clear();
  mapGaitNameToId_.clear();

  TiXmlHandle limbCoordinationHandle = handle;
  if(!tinyxml_tools::getChildHandle(limbCoordinationHandle, handle, "LimbCoordination")) { return false; }
  TiXmlHandle contactScheduleHandle = handle;
  if(!tinyxml_tools::getChildHandle(contactScheduleHandle, limbCoordinationHandle, "ContactSchedule")) { return false; }
  if(!tinyxml_tools::loadParameter(verbose_, contactScheduleHandle, "verbose")) { return false; }

  const TiXmlHandle gaitPatternHandle = tinyxml_tools::getChildHandle(contactScheduleHandle, "GaitPattern");

  // Default gait.
  std::string defaultGaitStr;
  if(!tinyxml_tools::loadParameter(defaultGaitStr, gaitPatternHandle, "default")) { return false; }
  MELO_DEBUG_STREAM(blue << "  Default gait is " << red << defaultGaitStr)

  // Balancing gait.
  if(!tinyxml_tools::loadParameter(balancingGaitName_, gaitPatternHandle, "balancing_gait")) { return false; }
  MELO_DEBUG_STREAM(blue << "  Balancing gait is " << red << balancingGaitName_)

  // Get names of valid gaits (for safety reasons).
  TiXmlHandle LimbCoordinationSafetyHandle = handle;
  if(!tinyxml_tools::getChildHandle(LimbCoordinationSafetyHandle, handle, "LimbCoordinationSafety")) { return false; }
  TiXmlHandle gaitHandle = handle;
  if(!tinyxml_tools::getChildHandle(gaitHandle, LimbCoordinationSafetyHandle, "GaitSelection")) { return false; }
  std::string validGaitPatternTypeStr;
  if(!tinyxml_tools::loadParameter(validGaitPatternTypeStr, gaitHandle, "type")) { return false; }
  auto validGaitNames =  contact_schedule::extractGaitNamesFromString(validGaitPatternTypeStr);
  validGaitNames.push_back(balancingGaitName_); // balancing gait is valid anyway

  // Read parameters for all gaits in the list.
  std::vector<TiXmlElement*> gaitElements;
  tinyxml_tools::getChildElements(gaitElements, gaitPatternHandle, "Gait");
  std::string gaitPatternTypeStr;
  bool didLoadBalancingGait = false;

  unsigned int gaitId = 0u;
  for (const auto& gait : gaitElements) {
    // Get name of the gait.
    if(!tinyxml_tools::loadParameter(gaitPatternTypeStr, gait, "type")) { return false; }

    // check if gait is valid.
    if (validGaitNames[0] != "all" &&
        std::find(validGaitNames.begin(), validGaitNames.end(), gaitPatternTypeStr) == validGaitNames.end()) {
      MELO_DEBUG_STREAM(yellow << "skip gait with name " << red << gaitPatternTypeStr)
      continue;
    }
    gaitDescription_.emplace_back(contact_schedule::periodic::GaitDescription());

    // Store the gait type and default gait index.
    mapGaitNameToId_.insert(std::make_pair(gaitPatternTypeStr, gaitId));
    if (gaitPatternTypeStr == defaultGaitStr){ defaultGaitId_ = gaitId; }

    MELO_DEBUG_STREAM(blue << "  Load parameters for " << red << "(" << gaitId << ") " << gaitPatternTypeStr << blue << " gait." << def)

    // Check if reversible
    bool isReversible;
    if(!tinyxml_tools::loadParameter(isReversible, gait, "reversible")) { return false; }
    gaitDescription_.back().setIsReversibleGait(isReversible);

    // Final gait.
    bool stopWithBalancingGait;
    if(!tinyxml_tools::loadParameter(stopWithBalancingGait, gait, "stop_with_balancing_gait")) { return false; }
    gaitDescription_.back().setStopWithBalancingGait(stopWithBalancingGait);

    // Perception.
    bool isPerceptiveGait;
    if(!tinyxml_tools::loadParameter(isPerceptiveGait, gait, "perceptive")) { return false; }
    gaitDescription_.back().setIsPerceptiveGait(isPerceptiveGait);

    // Nominal stride duration.
    double nominalStrideDuration;
    const TiXmlHandle strideHandle  = tinyxml_tools::getChildHandle(gait, "Stride");
    if(!tinyxml_tools::loadParameter(nominalStrideDuration, strideHandle, "nominal")) { return false; }
    gaitDescription_.back().setNominalStrideDuration(nominalStrideDuration);

    // Min stride duration and feedback gain for stride control.
    double strideFeedbackGain; double  minStrideDuration = 0.0;
    if(!tinyxml_tools::loadParameter(strideFeedbackGain, strideHandle, "fb_gain")) { return false; }
    if (strideFeedbackGain>0.0) {
      if(!tinyxml_tools::loadParameter(minStrideDuration, strideHandle, "min")) { return false; }
    }
    gaitDescription_.back().setStrideFeedbackParameters(strideFeedbackGain, minStrideDuration);

    // swing phase (optionally).
    double swingPhase = -1.0;
    tinyxml_tools::loadParameter(swingPhase, gait, "swingPhase", -1.0);

    // Load phase events.
    std::vector<TiXmlElement*> eventElements;
    for (const auto& legId : anymal_description::LegEnumIterator()) {
      if(!tinyxml_tools::getChildElements(eventElements, gait, anymal_description::mapLegEnumToString[legId])) {
        gaitDescription_.back().addGroundedLegId(legId);
        continue;
      }
      if(!loadEvents(eventElements, legId, swingPhase)) { return false; }
    }

    if(!gaitDescription_.back().createEventContainer()) { return false; }
    if(!gaitDescription_.back().getEventContainer().checkGait(gaitDescription_.back().getLegEnumsUsedForWalk())) { return false; }
    if(verbose_) { gaitDescription_.back().getEventContainer().print(); }
    ++gaitId;

    // Add balancing gait.
    if (gaitPatternTypeStr == balancingGaitName_) {
      mapGaitNameToId_.insert(std::make_pair(balancingGaitName_, gaitId));
      gaitDescription_.push_back(gaitDescription_.back());
      gaitDescription_.back().setStopWithBalancingGait(false);
      ++gaitId;
      didLoadBalancingGait = true;
    }
  }

  if (!didLoadBalancingGait) {
    MELO_FATAL_STREAM("[ContactSchedulePeriodic::loadParameters] Could not find gait called " << balancingGaitName_ << ".");
    return false;
  }

  return true;
}

bool ContactSchedulePeriodic::loadEvents(
    const std::vector<TiXmlElement*>& eventElements,
    contact_schedule::LegEnumAnymal legId,
    double swingPhase) {
  double liftOff;
  const bool globalSwingAvailable = (swingPhase>0.0);
  for (const auto& event : eventElements) {
    if(!tinyxml_tools::loadParameter(liftOff, event, "liftOff")) { return false; }
    if (!globalSwingAvailable) {
      if(!tinyxml_tools::loadParameter(swingPhase, event, "swingPhase")) { return false; }
    }
    gaitDescription_.back().push_back_phases(legId, liftOff, liftOff+swingPhase);
  }
  return true;
}

bool ContactSchedulePeriodic::advance(double dt) {
  // Update time since previous event.
  for (const auto& legId : anymal_description::LegEnumIterator()) {
    timeSincePreviousTouchDown_[legId] += dt;
    timeSincePreviousLiftOff_[legId]   += dt;
  }

  if (!updateStride(dt)) {
    MELO_WARN_STREAM("[ContactSchedulePeriodic::advance] Failed to update stride.");
    return false;
  }

  if (!updateTimeEvents(dt)) {
    MELO_WARN_STREAM("[ContactSchedulePeriodic::advance] Failed to update time events.");
    return false;
  }

  return true;
}

bool ContactSchedulePeriodic::updateStride(double dt) {
  if (!isLocked_ && isGaitPeriodic()) {
    if (getStrideDuration()>0.0) {
      stridePhase_ += dt/getStrideDuration();
      if (stridePhase_>=1.0) { stridePhase_ -= 1.0; }
    } else {
      MELO_WARN_STREAM("[ContactSchedulePeriodic::updateStride] Negative or zero stride duration of " << getStrideDuration() << ".");
      return false;
    }
  }
  return true;
}

bool ContactSchedulePeriodic::updateTimeEvents(double dt) {
  // Update time until next event.
  if(isGaitPeriodic()) {
    bool shouldLegBeGrounded; double phaseUntilNextLiftOff; double phaseUntilNextTouchDown;
    const auto legEnumsUsedForWalk = getLegEnumsUsedForWalk();
    if (!legEnumsUsedForWalk.empty()) {
      for (const auto& legId : legEnumsUsedForWalk) {
        getActiveGaitDescription().getGaitDescriptionForLeg(legId).getPhaseUntilNextEvents(
            shouldLegBeGrounded, phaseUntilNextLiftOff, phaseUntilNextTouchDown, stridePhase_
        );

        timeUntilNextLiftOff_[legId]   = phaseUntilNextLiftOff   * getStrideDuration();
        timeUntilNextTouchDown_[legId] = phaseUntilNextTouchDown * getStrideDuration();

        if(!updatePreviousEvent(shouldLegBeGrounded, legId)) { return false; }
      }
    }
  }

  return true;
}

bool ContactSchedulePeriodic::updatePreviousEvent(
    bool shouldLegBeGrounded,
    contact_schedule::LegEnumAnymal legId) noexcept {
  if (shouldBeGrounded_[legId] != shouldLegBeGrounded) { // contact state changes.
    if (shouldLegBeGrounded) { timeSincePreviousTouchDown_[legId] = 0.0; }
    else { timeSincePreviousLiftOff_[legId] = 0.0; }
    shouldBeGrounded_[legId] = shouldLegBeGrounded;
  }
  return true;
}

bool ContactSchedulePeriodic::updateActiveGait(unsigned int activeGaitId) {
  if(!mapGaitIdToName(activeGaitId, activeGaitName_)) {
    MELO_WARN_STREAM("[ContactSchedulePeriodic::updateActiveGait] Wrong gait index " << activeGaitId << ".")
    return false;
  }
  activeGaitId_   = activeGaitId;
  MELO_DEBUG_STREAM(blue << "[ContactSchedulePeriodic::updateActiveGait] Set new active gait " << red << activeGaitName_ << blue <<" ("<< activeGaitId_ << ")" << white)
  return true;
}

bool ContactSchedulePeriodic::updateActiveGait(const std::string& activeGaitName) {
  activeGaitName_ = activeGaitName;
  activeGaitId_   = mapGaitNameToId_[activeGaitName];
  MELO_DEBUG_STREAM(blue << "[ContactSchedulePeriodic::updateActiveGait] Set new active gait " << red << activeGaitName_ << blue <<" ("<< activeGaitId_ << ")"<< white)
  return true;
}

const std::string& ContactSchedulePeriodic::getActiveGaitName() const noexcept {
  return activeGaitName_;
}

unsigned int ContactSchedulePeriodic::getActiveGaitId() const noexcept {
  return activeGaitId_;
}

const std::map<std::string, unsigned int>& ContactSchedulePeriodic::getMapGaitNameToId() const noexcept {
  return mapGaitNameToId_;
}

bool ContactSchedulePeriodic::mapGaitIdToName(unsigned int gaitId, std::string& gaitName) const noexcept {
  for (const auto& gaitNameIdPar : mapGaitNameToId_) {
    if (gaitNameIdPar.second == gaitId) {
      gaitName = gaitNameIdPar.first;
      return true;
    }
  }
  return false;
}

bool ContactSchedulePeriodic::isValidGait(const std::string& gaitName) const noexcept {
  // Balancing gait may not be used for locomotion.
  if (gaitName == balancingGaitName_) { return false; }

  // Check if gait name is in the gait map.
  for (const auto& gaitNameIdPar : mapGaitNameToId_) {
    if (gaitNameIdPar.first == gaitName) { return true; }
  }
  return false;
}

bool ContactSchedulePeriodic::isValidGait(unsigned int gaitId) const noexcept {
  std::string gaitName;
  if(!mapGaitIdToName(gaitId, gaitName)) { return false; }
  return isValidGait(gaitName);
}

double ContactSchedulePeriodic::getStrideDuration() const noexcept {
  return getActiveGaitDescription().getStrideDuration();
}

double ContactSchedulePeriodic::getNominalStrideDuration() const noexcept {
  return getActiveGaitDescription().getNominalStrideDuration();
}

void ContactSchedulePeriodic::setActiveNominalStrideDuration(double nominalStrideDuration) noexcept {
  getActiveGaitDescription().setNominalStrideDuration(nominalStrideDuration);
}

const contact_schedule::periodic::GaitDescription& ContactSchedulePeriodic::getActiveGaitDescription() const {
  assert(activeGaitId_<gaitDescription_.size());
  return gaitDescription_[activeGaitId_];
}

contact_schedule::periodic::GaitDescription& ContactSchedulePeriodic::getActiveGaitDescription() {
  assert(activeGaitId_<gaitDescription_.size());
  return gaitDescription_[activeGaitId_];
}

bool ContactSchedulePeriodic::stopActiveGaitWithBalancingGait() const noexcept {
  return getActiveGaitDescription().stopWithBalancingGait();
}

double ContactSchedulePeriodic::getIsPerceptiveGait(unsigned int gaitId) const {
  assert(gaitId<gaitDescription_.size());
  return gaitDescription_[gaitId].getIsPerceptiveGait(); 
}

bool ContactSchedulePeriodic::isActiveGaitReversible() const noexcept {
  return getActiveGaitDescription().getIsReversibleGait();
}

const std::vector<contact_schedule::LegEnumAnymal>& ContactSchedulePeriodic::getLegEnumsUsedForWalk() const {
  return getActiveGaitDescription().getLegEnumsUsedForWalk();
}

const std::vector<contact_schedule::LegEnumAnymal>& ContactSchedulePeriodic::getLegEnumsUsedForStand() const {
  return getActiveGaitDescription().getLegEnumsUsedForStand();
}

bool ContactSchedulePeriodic::isLegIdGrounded(contact_schedule::LegEnumAnymal legId) const {
  return getActiveGaitDescription().isLegIdGrounded(legId);
}

}  // end namespace loco
