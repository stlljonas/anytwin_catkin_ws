/*
 * contact_schedule.hpp
 *
 *  Created on: April 18, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// loco
#include "loco/gait_pattern/contact_schedule_anymal.hpp"

// std utils
#include "map"
#include "std_utils/std_utils.hpp"
#include "vector"

namespace loco {
namespace contact_schedule {

/*
 * Status
 *  > Stand:            Gait is in stance mode.
 *  > Walk:             Gait is in walk mode.
 *  > SwitchGait:       Gait switcher is active. Switch between to different gaits.
 *  > SwitchToStand:    Switch from walk to stand.
 *  > SwitchToWalk:     Switch from stand to walk.
 *  > ExecuteOneCycle:  Robot is standing. Execute one gait cycle in place.
 *  > ForceStance:      Gait is forced to stay in stance mode.
 */
CONSECUTIVE_ENUM(Status, Stand, Walk, SwitchGait, SwitchToStand, SwitchToWalk, ExecuteOneCycle, ForceStance, Undefined)
static std::map<Status, std::string> statusMap = {{Status::Stand, "Stand"},
                                                  {Status::Walk, "Walk"},
                                                  {Status::SwitchGait, "SwitchGait"},
                                                  {Status::SwitchToStand, "SwitchToStand"},
                                                  {Status::SwitchToWalk, "SwitchToWalk"},
                                                  {Status::ExecuteOneCycle, "ExecuteOneCycle"},
                                                  {Status::ForceStance, "ForceStance"},
                                                  {Status::Undefined, "Undefined"},
                                                  {Status::SIZE, "SIZE"}};
inline std::ostream& operator<<(std::ostream& out, const Status value) {
  return out << statusMap[value];
}

/*
 * Contact Schedule events:
 *  > LiftOff:    leg is expected to lift off at phase.
 *  > TouchDown:  leg is expected to touch down at phase.
 *  > EndOfGait:  specifies the very last event.
 */
CONSECUTIVE_ENUM(ContactEvent, LiftOff, TouchDown, EndOfGait, Undefined)
static std::map<ContactEvent, std::string> contactEventsMap = {{ContactEvent::LiftOff, "LiftOff"},
                                                               {ContactEvent::TouchDown, "TouchDown"},
                                                               {ContactEvent::Undefined, "Undefined"},
                                                               {ContactEvent::EndOfGait, "EndOfGait"},
                                                               {ContactEvent::SIZE, "SIZE"}};
inline std::ostream& operator<<(std::ostream& out, const ContactEvent value) {
  return out << contactEventsMap[value];
}

/*
 * Event horizon status:
 *  > LiftOffAndTouchDown:  next swing time and next stance time is known.
 *  > TouchDown:            only next touch-down time is known.
 *  > LiftOff:              only next lift-off time is known.
 *  > None:                 no information  about future events is known.
 */
CONSECUTIVE_ENUM(EventHorizonStatus, LiftOffAndTouchDown, TouchDown, LiftOff, None)
static std::map<EventHorizonStatus, std::string> eventHorizonMap = {{EventHorizonStatus::LiftOffAndTouchDown, "LiftOffAndTouchDown"},
                                                                    {EventHorizonStatus::TouchDown, "TouchDown"},
                                                                    {EventHorizonStatus::LiftOff, "LiftOff"},
                                                                    {EventHorizonStatus::None, "None"},
                                                                    {EventHorizonStatus::SIZE, "SIZE"}};
inline std::ostream& operator<<(std::ostream& out, const EventHorizonStatus value) {
  return out << eventHorizonMap[value];
}

/*
 * Contact Schedule Switch status
 *  > Switched:  transition could be established.
 *  > NotFound:  Desired gait not found.
 *  > Error:     something went wrong.
 *  > Running:   Switcher is already running. Try to switch later.
 */
CONSECUTIVE_ENUM(ContactScheduleSwitchStatus, Switched, NotFound, Error, Running, Undefined)

}  // namespace contact_schedule
}  // namespace loco
