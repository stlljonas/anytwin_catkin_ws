/*
 * GaitDescription.hpp
 *
 *  Created on: April 18, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// anymal_description
#include <anymal_description/LegEnum.hpp>

// motion_generation
#include "motion_generation/GaitDescriptionLeg.hpp"
#include "motion_generation/EventContainerPeriodic.hpp"

//loco
#include "loco/gait_pattern/contact_schedule.hpp"

namespace loco {
namespace contact_schedule {
namespace periodic {


class GaitDescription {
public:
  GaitDescription():
    gaitDescriptionLegs_(),
    eventContainer_(),
    nominalStrideDuration_(0.0),
    strideFeedback_(0.0),
    minStrideDuration_(0.0) ,
    strideFeedbackGain_(0.0),
    stopWithBalancingGait_(false),
    isReversible_(false),
    legEnumsUsedForWalk_(),
    legEnumsUsedForStand_(),
    isPerceptiveGait_(false) {

    // By default, all legs are used for walking.
    legEnumsUsedForWalk_.clear();
    legEnumsUsedForWalk_.reserve(static_cast<unsigned int>(LegEnumAnymal::SIZE));
    legEnumsUsedForStand_.clear();
    for (const auto& legId : anymal_description::LegEnumIterator()) {
      legEnumsUsedForWalk_.push_back(legId);
    }
  }

  ~GaitDescription() = default;

  GaitDescription(const GaitDescription&) = default;
  GaitDescription& operator=(const GaitDescription&) = default;
  GaitDescription(GaitDescription&&) = default;
  GaitDescription& operator=(GaitDescription&&) = default;

  void clear() noexcept {
    for (auto& gaitDescriptionLeg : gaitDescriptionLegs_) { gaitDescriptionLeg.clear(); }
    eventContainer_.clear();
    nominalStrideDuration_ = 0.0;
  }

  bool changeDirectionOfMotion() {
    const auto gaitDescriptionLegs = gaitDescriptionLegs_;
    if (!legEnumsUsedForWalk_.empty()) {
      for (const auto& legId : legEnumsUsedForWalk_) {
        const auto legIdMirrored = transformLegIdForDirectionChange(legId);
        if (!isLegIdGrounded(legIdMirrored)) {
          gaitDescriptionLegs_[legId].setLiftOffPhaseEvents(gaitDescriptionLegs[legIdMirrored].getLiftOffPhaseEvents());
          gaitDescriptionLegs_[legId].setTouchDownPhaseEvents(gaitDescriptionLegs[legIdMirrored].getTouchDownPhaseEvents());
        }
      }
    }
    return createEventContainer();
  }

  const GaitDescriptionLeg& getGaitDescriptionForLeg(LegEnumAnymal legId) const noexcept {
    if (isLegIdGrounded(legId)) {
      MELO_WARN_STREAM("[GaitDescription::getGaitDescriptionForLeg] leg is grounded!");
    }
    return gaitDescriptionLegs_[legId];
  }

  double getNominalStrideDuration() const noexcept {
    return nominalStrideDuration_;
  }

  void setNominalStrideDuration(double nominalStrideDuration) noexcept {
    nominalStrideDuration_ = nominalStrideDuration;
  }

  void push_back_phases(LegEnumAnymal legId, double liftOffPhase, double touchDownPhase) {
    if (isLegIdGrounded(legId)) {
      MELO_WARN_STREAM("[GaitDescription::push_back_phases] leg is grounded!");
    }
    gaitDescriptionLegs_[legId].push_back_phases(liftOffPhase, touchDownPhase);
  }

  bool createEventContainer() {
    eventContainer_.clear();
    if (!legEnumsUsedForWalk_.empty()) {
      for (const auto& legId : legEnumsUsedForWalk_) {
        // Add lift-off events.
        for (auto liftOffPhase : gaitDescriptionLegs_[legId].getLiftOffPhaseEvents()) {
          robot_utils::mapToUnitInterval(liftOffPhase);
          if(!eventContainer_.addEvent(legId, ContactEvent::LiftOff, liftOffPhase)) { return false; }
        }

        // Add touch-down events.
        for (auto touchDownPhase : gaitDescriptionLegs_[legId].getTouchDownPhaseEvents()) {
          robot_utils::mapToUnitInterval(touchDownPhase);
          if(!eventContainer_.addEvent(legId, ContactEvent::TouchDown, touchDownPhase)) { return false; }
        }
      }
    }
    return true;
  }

  const EventContainerPeriodic& getEventContainer() const noexcept {
    return eventContainer_;
  }

  void print() const {
    if (!legEnumsUsedForWalk_.empty()) {
      for (const auto& legId : legEnumsUsedForWalk_) {
        std::cout << "leg " << legId << "\n   lift-off: ";
        for (const auto event : gaitDescriptionLegs_[legId].getLiftOffPhaseEvents()) {
          std::cout << event << " ";
        }
        std::cout << "\n   touch-down: ";
        for (const auto event : gaitDescriptionLegs_[legId].getTouchDownPhaseEvents()) {
          std::cout << event << " ";
        }
        std::cout << std::endl;
      }
    }
    if (!legEnumsUsedForStand_.empty()) {
      for (const auto& legId : legEnumsUsedForStand_) {
        std::cout << "leg " << legId << "\n   Leg is grounded! " << std::endl;
      }
    }
  }

  //! Find the first lift-off/touch-down event in the gait.
  double mapLegIndexToFirstEventPhase(LegEnumAnymal legIdAtEvent, ContactEvent contactEvent = ContactEvent::LiftOff) const noexcept {
    if (isLegIdGrounded(legIdAtEvent)) {
      std::cout << "[GaitDescription::mapLegIndexToFirstEventPhase] leg is grounded!\n";
      return -1.0;
    }
    switch (contactEvent) {
      case ContactEvent::LiftOff : {
        const auto liftOffEvents = gaitDescriptionLegs_[legIdAtEvent].getSortedLiftOffEvents();
        if (liftOffEvents.empty()) { return -1.0; }
        return liftOffEvents.front();
      }

      case ContactEvent::TouchDown : {
        const auto touchDownEvents = gaitDescriptionLegs_[legIdAtEvent].getSortedTouchDownEvents();
        if (touchDownEvents.empty()) { return -1.0; }
        return touchDownEvents.front();
      }

      default : {
        return -1.0;
      }
    }
  }

  void setStrideFeedbackParameters(double strideFeedbackGain, double minStrideDuration) noexcept {
    strideFeedbackGain_ = strideFeedbackGain;
    minStrideDuration_  = minStrideDuration;
  }

  void setStrideFeedback(double strideFeedback) noexcept {
    strideFeedback_ = strideFeedback;
  }

  double getStrideFeedback() const noexcept {
    return strideFeedback_;
  }

  double getStrideDuration() const noexcept {
    if (strideFeedbackGain_>0.0) {
      const double strideDuration = nominalStrideDuration_ + strideFeedback_*strideFeedbackGain_;
      if (strideDuration<minStrideDuration_) { return minStrideDuration_; }
      return strideDuration;
    }
    return nominalStrideDuration_;
  }

  void setStopWithBalancingGait(bool stopWithBalancingGait) noexcept {
    stopWithBalancingGait_ = stopWithBalancingGait;
  }

  bool stopWithBalancingGait() const noexcept {
    return stopWithBalancingGait_;
  }

  void setIsPerceptiveGait(bool isPerceptiveGait) noexcept {
    isPerceptiveGait_ = isPerceptiveGait;
  }

  bool getIsPerceptiveGait() const noexcept {
    return isPerceptiveGait_;
  }
  
  void setIsReversibleGait(bool isReversible) noexcept {
    isReversible_ = isReversible;
  }

  bool getIsReversibleGait() const noexcept {
    return isReversible_;
  }

  void addGroundedLegId(LegEnumAnymal legId) {
    legEnumsUsedForStand_.push_back(legId);
    auto iter = std::find(legEnumsUsedForWalk_.begin(), legEnumsUsedForWalk_.end(), legId);
    legEnumsUsedForWalk_.erase(iter);
  }

  const std::vector<contact_schedule::LegEnumAnymal>& getLegEnumsUsedForWalk() const noexcept {
    return legEnumsUsedForWalk_;
  }

  const std::vector<contact_schedule::LegEnumAnymal>& getLegEnumsUsedForStand() const noexcept {
    return legEnumsUsedForStand_;
  }

  bool isLegIdGrounded(contact_schedule::LegEnumAnymal legId) const {
    if (legEnumsUsedForStand_.empty()) {
      return false;
    } else {
      return std::find(legEnumsUsedForStand_.begin(), legEnumsUsedForStand_.end(), legId) != legEnumsUsedForStand_.end();
    }
  }

private:

  //! Description of the gait in terms of lift-off and touch-down phases.
  std_utils::EnumArray<LegEnumAnymal, GaitDescriptionLeg> gaitDescriptionLegs_;

  //! Description of the gait in terms of phase events.
  EventContainerPeriodic eventContainer_;

  //! Stride duration (uncontrolled, unperturbed).
  double nominalStrideDuration_;

  //! Perturbation for the stride duration.
  double strideFeedback_;

  //! Min value of the stride duration.
  double minStrideDuration_;

  //! Proportional feedback gain for stride control.
  double strideFeedbackGain_;

  //! If true, the gait is stopped with the balancing gait.
  bool stopWithBalancingGait_;

  //! If true, the gait is reversible.
  bool isReversible_;

  //! Leg indices corresponding to legs that are used for stepping.
  std::vector<contact_schedule::LegEnumAnymal> legEnumsUsedForWalk_;

  //! Leg indices corresponding to legs that are grounded (e.g., driving in the case of wheeled-legged robots).
  std::vector<contact_schedule::LegEnumAnymal> legEnumsUsedForStand_;

  //! If true elevation map is used with this gait.
  bool isPerceptiveGait_;
};


} // namespace contact_schedule
} // namespace periodic
} // namespace loco
