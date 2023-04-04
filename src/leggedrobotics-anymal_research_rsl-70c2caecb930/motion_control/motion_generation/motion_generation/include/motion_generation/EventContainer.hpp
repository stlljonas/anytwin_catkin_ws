/*
 * EventContainer.hpp
 *
 *  Created on: May 1, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// loco
#include "loco/gait_pattern/contact_schedule.hpp"

// message logger
#include "message_logger/message_logger.hpp"

// robot utils
#include "robot_utils/math/math.hpp"

namespace loco {
namespace contact_schedule {

struct Event {
  //! Index of the leg.
  LegEnumAnymal legId_;

  //! Type of the event.
  ContactEvent contactEvent_;

  //! True if event corresponds to desired gait (relevant only during gait transition).
  bool isDesiredGait_;

  Event() :
    legId_(LegEnumAnymal::SIZE),
    contactEvent_(ContactEvent::SIZE),
    isDesiredGait_(false)
  { }

  Event(LegEnumAnymal legId, ContactEvent contactEvent, bool isDesiredGait = false) :
    legId_(legId),
    contactEvent_(contactEvent),
    isDesiredGait_(isDesiredGait)
  { }

  bool operator ==(const Event& event) const {
    if (legId_ != event.legId_) { return false; }
    if (contactEvent_ != event.contactEvent_) { return false; }
    return true;
  }

  void print() const {
    constexpr auto streamWidth = 20;
    std::stringstream msg;
    msg << std::left << std::endl << "-------------" << std::endl;
    msg << std::setw(streamWidth) << "event"
        << std::setw(streamWidth) << "leg"
        << std::endl;
    msg << std::setw(streamWidth) << contact_schedule::contactEventsMap[contactEvent_]
        << std::setw(streamWidth) << legId_
        << std::endl;
    msg << "-------------" << std::endl;
    MELO_INFO_STREAM(msg.str());
  }
};

//! Map that relates a time with an event.
using Events       = std::vector<Event>;
using ListOfEvents = std::map<double, std::vector<Event>>;

//! Check if two events are identical.
static inline bool areEventsIdentical(const Events& events1, const Events& events2) {
  if (events1.size() != events2.size()) { return false; }

  for (const auto& event1: events1) {
    if (std::find(events2.begin(), events2.end(), event1) == events2.end()) {
      return false;
    }
  }
  return true;
}

class EventContainer {
public:
  EventContainer() :
    listOfEvents_()
  {

  }

  virtual ~EventContainer() = default;

  void clear() noexcept {
    listOfEvents_.clear();
  }

  //! Return list of events.
  const ListOfEvents& getListOfEvents() const noexcept {
    return listOfEvents_;
  }

  void setListOfEvents(const ListOfEvents& listOfEvents) noexcept {
    listOfEvents_ = listOfEvents;
  }

  //! Extract list of events within predefined time interval.
  bool extractListOfEventsInTimeWindow(EventContainer& eventContainer, double timeStart, double timeEnd) const {
    eventContainer.clear();
    ListOfEvents listOfEventsInTimeInterval;

    for (const auto& events : listOfEvents_) {
      if (events.first>timeStart && robot_utils::isEqualOrSmallerThan(events.first, timeEnd)) {
        listOfEventsInTimeInterval.insert(std::make_pair(events.first-timeStart, events.second));
      }
    }
    eventContainer.setListOfEvents(listOfEventsInTimeInterval);
    return true;
  }

  //! Extract list of events for times larger than timeStart and s.t. each leg has at most one lift-off
  // and one touch-down event.
  bool extractListOfEventsFullCycle(EventContainer& eventContainer, double timeStart) const {
    eventContainer.clear();
    ListOfEvents listOfEventsInTimeInterval;
    std_utils::EnumArray<LegEnumAnymal, bool> didLiftOff(false);
    std_utils::EnumArray<LegEnumAnymal, bool> didTouchedDown(false);

    for (const auto& events : listOfEvents_) {
      if (events.first>timeStart) {
        double timeLeftInEvent = -1.0;
        ListOfEvents::iterator eventIt;

        for(const auto& event : events.second) {
          // Check if a new event appears.
          if ((event.contactEvent_ == ContactEvent::LiftOff && !didLiftOff[event.legId_]) ||
              (event.contactEvent_ == ContactEvent::TouchDown && !didTouchedDown[event.legId_])) {

            // Add event time.
            if (timeLeftInEvent<0.0) {
              timeLeftInEvent = events.first-timeStart;
              eventIt = listOfEventsInTimeInterval.insert( std::make_pair(timeLeftInEvent, contact_schedule::Events()) ).first;
            }

            // Add event.
            eventIt->second.push_back(event);

            // Remember.
            if (event.contactEvent_ == ContactEvent::LiftOff)   { didLiftOff[event.legId_] = true; }
            if (event.contactEvent_ == ContactEvent::TouchDown) { didTouchedDown[event.legId_] = true; }
          }

        }
      }
    }
    eventContainer.setListOfEvents(listOfEventsInTimeInterval);
    return true;
  }

  //! Returns time until next lift-off w.r.t. to time. In case no event is found, return -1.0.
  double getTimeUntilNextLiftOffAtTime(LegEnumAnymal legId, double time = 0.0) const {
    assert(time>=0.0);
    for (const auto& events : listOfEvents_) {
      if (robot_utils::isEqualOrSmallerThan(events.first, time)) { continue; }
      for (const auto& event : events.second) {
        if (event.legId_ == legId && event.contactEvent_ == ContactEvent::LiftOff) {
          return (events.first-time);
        }
      }
    }
    return -1.0;
  }

  //! Returns time until next touch-down w.r.t. to time. In case no event is found, return -1.0.
  double getTimeUntilNextTouchDownAtTime(LegEnumAnymal legId, double time = 0.0) const {
    assert(time>=0.0);
    for (const auto& events : listOfEvents_) {
      if (robot_utils::isEqualOrSmallerThan(events.first, time)) { continue; }
      for (const auto& event : events.second) {
        if (event.legId_ == legId && event.contactEvent_ == ContactEvent::TouchDown) {
          return (events.first-time);
        }
      }
    }
    return -1.0;
  }

  //! Add event to the list of time events.
  bool addEvent(
      LegEnumAnymal legId,
      ContactEvent contactEvent,
      double timeUntilEvent,
      bool isDesiredGait = false) {
    assert(timeUntilEvent>=0.0);

    // If time offset does not exist in list of events, add a new event.
    ListOfEvents::iterator eventIt;
    if (!isTimeInListOfEvents(timeUntilEvent, eventIt)) {
      eventIt = listOfEvents_.insert( std::make_pair(timeUntilEvent, contact_schedule::Events()) ).first;
    }

    // Add event.
    eventIt->second.emplace_back(contact_schedule::Event(legId, contactEvent, isDesiredGait));
    return true;
  }

  //! True there exists an element in the list of events with time.
  bool isTimeInListOfEvents(double time, ListOfEvents::iterator& eventIt) {
    eventIt = std::find_if(listOfEvents_.begin(), listOfEvents_.end(),
        [time] (const ListOfEvents::value_type& event) {
          return robot_utils::areNear(event.first, time);
        }
    );
    return (eventIt != listOfEvents_.end());
  }

  //! Print list of events to the console.
  virtual void print(double rate = -1.0) const {
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

    msg << "-------------" << std::endl;
    if (rate>0.0) {
      MELO_INFO_THROTTLE_STREAM(rate, msg.str());
    } else {
      MELO_INFO_STREAM(msg.str());
    }
  }

protected:

  //! Map that relates a time with an event.
  ListOfEvents listOfEvents_;

};

}
} // namespace loco
