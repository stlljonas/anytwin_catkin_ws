/*
 * EventContainerPeriodic.hpp
 *
 *  Created on: May 1, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// anymal_description
#include <anymal_description/LegEnum.hpp>

// motion_generation
#include "motion_generation/EventContainer.hpp"

namespace loco {
namespace contact_schedule {
namespace periodic {

class EventContainerPeriodic : public EventContainer {
public:
  EventContainerPeriodic() :
    EventContainer()
  {  }

  ~EventContainerPeriodic() override = default;

  //! True if the leg should swing at stride phase 0.
  bool shouldLegSwingAtStart(LegEnumAnymal legId) const noexcept {
    for (const auto& events : listOfEvents_) {
      for (const auto& event : events.second) {
        if (event.legId_ == legId) {
          if (event.contactEvent_ == ContactEvent::TouchDown) { return true; }
          else if (event.contactEvent_ == ContactEvent::LiftOff) { return false; }
        }
      }
    }

    MELO_WARN_STREAM("[EventContainerPeriodic::numOfSwingingLegsAtStart] Could not find first event for leg " << legId << ".");
    return false;

  }

  //! Returns number of swinging legs if the gait is started at stride phase 0.
  unsigned int numOfSwingingLegsAtStart(const std::vector<contact_schedule::LegEnumAnymal>& legEnumsUsedForWalk) const noexcept {
    unsigned int numOfSwingingLegs = 0u;

    for (const auto& legId : legEnumsUsedForWalk) {
      if (shouldLegSwingAtStart(legId)) { ++numOfSwingingLegs; }
    }
    return numOfSwingingLegs;
  }

  /*! Returns the largest full stance phase (all legs are grounded). If there is no full stance phase in the gait,
   * this function returns -1.0.
   */
  double findLargestFullStancePhase(const std::vector<contact_schedule::LegEnumAnymal>& legEnumsUsedForWalk) const {
    double maxFullStancePhase = -1.0;
    int numOfSwingingLegs = numOfSwingingLegsAtStart(legEnumsUsedForWalk);

    for (auto eventIter = listOfEvents_.begin(); eventIter!=listOfEvents_.end(); ++eventIter) {
      assert(eventIter->first<=1.0 && eventIter->first>=0.0);

      for (const auto& event : eventIter->second) {
        if (event.contactEvent_==ContactEvent::LiftOff) {  ++numOfSwingingLegs; }
        else if (event.contactEvent_==ContactEvent::TouchDown) { --numOfSwingingLegs; }

        if (numOfSwingingLegs<0 || numOfSwingingLegs>static_cast<int>(numOfLegs)) {
          MELO_WARN_STREAM("[EventContainerPeriodic::findLargestFullStancePhase] Wrong number of swinging legs " << numOfSwingingLegs << ".");
          return -1.0;
        }
      }

      // We found a full stance phase if non of the legs are swinging.
      if (numOfSwingingLegs == 0) {
        auto nextEventIter = std::next(eventIter);
        if (nextEventIter!=listOfEvents_.end()) {
          maxFullStancePhase = std::fmax(maxFullStancePhase, nextEventIter->first - eventIter->first);
        } else {
          maxFullStancePhase = std::fmax(maxFullStancePhase, 1.0-eventIter->first + listOfEvents_.begin()->first);
        }
      }
    }

    return maxFullStancePhase;
  }

  /*! Returns the largest full flight phase (all legs are swinging). If there is no full flight phase in the gait,
   * this function returns -1.0.
   */
  double findLargestFullFlightPhase(const std::vector<contact_schedule::LegEnumAnymal>& legEnumsUsedForWalk) const {
    double maxFullStancePhase = -1.0;
    int numOfSwingingLegs = numOfSwingingLegsAtStart(legEnumsUsedForWalk);

    for (auto eventIter = listOfEvents_.begin(); eventIter!=listOfEvents_.end(); ++eventIter) {
      assert(eventIter->first<=1.0 && eventIter->first>=0.0);

      for (const auto& event : eventIter->second) {
        if (event.contactEvent_==ContactEvent::LiftOff) {  ++numOfSwingingLegs; }
        else if (event.contactEvent_==ContactEvent::TouchDown) { --numOfSwingingLegs; }

        if (numOfSwingingLegs<0 || numOfSwingingLegs>static_cast<int>(numOfLegs)) {
          MELO_WARN_STREAM("[EventContainerPeriodic::findLargestFullFlightPhase] Wrong number of swinging legs " << numOfSwingingLegs << ".");
          return -1.0;
        }
      }

      // We found a full flight phase if all of the legs are swinging.
      if (numOfSwingingLegs == contact_schedule::numOfLegs) {
        auto nextEventIter = std::next(eventIter);
        if (nextEventIter!=listOfEvents_.end()) {
          maxFullStancePhase = std::fmax(maxFullStancePhase, nextEventIter->first - eventIter->first);
        } else {
          maxFullStancePhase = std::fmax(maxFullStancePhase, 1.0-eventIter->first + listOfEvents_.begin()->first);
        }
      }
    }

    return maxFullStancePhase;
  }

  //! Find the next lift-off legs relative to legId and corresponding phase in the sequence of lift-off events.
   bool findNextLiftOffLegs(LegEnumAnymal legId, std::vector<LegEnumAnymal>& nextLifOffLegs, double& phase) const {
     if (listOfEvents_.size() <= 1) { return false; }

     nextLifOffLegs.clear();
     phase = -1.0;
     bool foundLiftOff = false;
     bool foundNextLiftOff = false;

     for (unsigned int gaitId=0u; gaitId<2u; ++gaitId) {
       for (const auto& events : listOfEvents_) {
        for (const auto& event : events.second) {

          // Seek for the first lift-off event for legId.
          if (!foundLiftOff && event.legId_ == legId && event.contactEvent_==ContactEvent::LiftOff) {
            foundLiftOff = true;
            break;
          }

          // Seek for the leg index that lifts off next.
          else if (foundLiftOff && event.contactEvent_==ContactEvent::LiftOff) {
            nextLifOffLegs.push_back(event.legId_);
            phase = events.first;
            foundNextLiftOff = true;
          }
        }

        if (foundNextLiftOff) { return true; }
      }
    }

    return false;
  }

  bool checkGait(const std::vector<contact_schedule::LegEnumAnymal>& legEnumsUsedForWalk) const noexcept {
    // Check if phases are in the interval [0, 1).
    for (const auto& events : listOfEvents_) {

      if (events.first < 0.0 || events.first > 1.0) {
        MELO_WARN_STREAM("[EventContainerPeriodic::checkGait] Invalid phase " << events.first << ".");
        return false;
      }
    }

    // Check if after a lift-off follows a touch-down event.
    for (const auto& legId : legEnumsUsedForWalk) {
      ContactEvent expectedEvent = (shouldLegSwingAtStart(legId) ? ContactEvent::TouchDown : ContactEvent::LiftOff);

      for (const auto &events : listOfEvents_) {
        for (const auto &event : events.second) {
          if (legId == event.legId_) {

            if (event.contactEvent_ != expectedEvent) {
              MELO_WARN_STREAM("[EventContainerPeriodic::checkGait] Invalid contact event " << contactEventsMap[event.contactEvent_] << ".");
              return false;
            }

            if (event.contactEvent_ == ContactEvent::LiftOff) { expectedEvent = ContactEvent::TouchDown; }
            else if (event.contactEvent_ == ContactEvent::TouchDown) { expectedEvent = ContactEvent::LiftOff; }
          }
        }
      }
    }

    return true;
  }

  void print(double rate = -1.0) const override {
    constexpr auto streamWidth = 20;
    std::stringstream msg;
    msg << std::left << std::endl << "-------------" << std::endl;
    msg << std::setw(streamWidth) << "d[s]"
        << std::setw(streamWidth) << "event"
        << std::setw(streamWidth) << "leg"
        << std::setw(streamWidth) << "id"
        << std::endl;
    for (const auto& events : listOfEvents_) {
      for (const auto& event : events.second) {
        msg << std::setw(streamWidth) << std::to_string(events.first)
            << std::setw(streamWidth) << contact_schedule::contactEventsMap[event.contactEvent_]
            << std::setw(streamWidth) << event.legId_
            << std::setw(streamWidth) << static_cast<unsigned int>(event.legId_)
            << std::endl;
      }
    }

    std::vector<contact_schedule::LegEnumAnymal> legEnumsUsedForWalk;
    legEnumsUsedForWalk.reserve(static_cast<unsigned int>(LegEnumAnymal::SIZE));
    for (const auto& legId : anymal_description::LegEnumIterator()) {
      legEnumsUsedForWalk.push_back(legId);
    }

    msg << "num of swinging legs at gait start = " << numOfSwingingLegsAtStart(legEnumsUsedForWalk) << std::endl;
    msg << "full stance phase = " << findLargestFullStancePhase(legEnumsUsedForWalk) << std::endl;
    msg << "full flight phase = " << findLargestFullFlightPhase(legEnumsUsedForWalk) << std::endl;

    msg << "-------------" << std::endl;
    if (rate > 0.0) {
      MELO_INFO_THROTTLE_STREAM(rate, msg.str());
    } else {
      MELO_INFO_STREAM(msg.str());
    }
  }

};

} // namespace periodic
} // namespace contact_schedule
} // namespace loco
