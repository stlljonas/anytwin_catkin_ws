/*
 * StateSwitcher.cpp
 *
 *  Created on: Oct 5, 2014
 *      Author: Dario Bellicoso
 */

// loco
#include "loco/state_switcher/StateSwitcher.hpp"

// signal logger
#include "signal_logger/signal_logger.hpp"

namespace loco {

StateSwitcher::StateSwitcher() : StateSwitcherBase(), limbState_(static_cast<int>(States::Init)) {
  // Initialize name map.
  statesEnumNameMap_.insert(std::pair<States, std::string>(States::Init, "init"));
  statesEnumNameMap_.insert(std::pair<States, std::string>(States::StanceNormal, "stance_normal"));
  statesEnumNameMap_.insert(std::pair<States, std::string>(States::StanceSlipping, "stance_slipping"));
  statesEnumNameMap_.insert(std::pair<States, std::string>(States::StanceLostContact, "stance_lost_contact"));
  statesEnumNameMap_.insert(std::pair<States, std::string>(States::SwingNormal, "swing_normal"));
  statesEnumNameMap_.insert(std::pair<States, std::string>(States::SwingLateLiftOff, "swing_late_lift_off"));
  statesEnumNameMap_.insert(std::pair<States, std::string>(States::SwingEarlyTouchDown, "swing_early_touchdown"));
  statesEnumNameMap_.insert(std::pair<States, std::string>(States::SwingExpectingContact, "swing_expecting_contact"));
  statesEnumNameMap_.insert(std::pair<States, std::string>(States::SwingBumpedIntoObstacle, "swing_bumped_into_obstacle"));
}

bool StateSwitcher::initialize(double dt) {
  limbState_ = static_cast<int>(States::Init);

  return true;
}

StateSwitcher::States StateSwitcher::getState() const {
  return static_cast<States>(limbState_);
}

void StateSwitcher::setState(States state) {
  limbState_ = static_cast<int>(state);
}

bool StateSwitcher::addVariablesToLog(const std::string& ns) const {
  signal_logger::add(limbState_, "limbState", ns);
  return true;
}

const std::string& StateSwitcher::getStateNameFromEnum(StateSwitcher::States state) const {
  return statesEnumNameMap_.at(state);
}

const std::string& StateSwitcher::getCurrentStateName() const {
  return statesEnumNameMap_.at(static_cast<States>(limbState_));
}

} /* namespace loco */