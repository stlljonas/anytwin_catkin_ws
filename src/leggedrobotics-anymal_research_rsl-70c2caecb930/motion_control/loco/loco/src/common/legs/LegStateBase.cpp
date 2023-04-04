/*
 * LegStateBase.cpp
 *
 *  Created on: Feb 26, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

// loco
#include "loco/common/legs/LegStateBase.hpp"

namespace loco {

LegStateBase::LegStateBase() : isNow_(false), lastStateWasEarly_(false), lastStateWasLate_(false), stateChangedAtTime_(0.0), phase_(0.0) {}

void LegStateBase::setIsNow(bool isNow) {
  isNow_ = isNow;
}
bool LegStateBase::isNow() const {
  return isNow_;
}

void LegStateBase::setLastStateWasEarly(bool wasEarly) {
  lastStateWasEarly_ = wasEarly;
}
bool LegStateBase::lastStateWasEarly() const {
  return lastStateWasEarly_;
}

void LegStateBase::setLastStateWasLate(bool wasLate) {
  lastStateWasLate_ = wasLate;
}
bool LegStateBase::lastStateWasLate() const {
  return lastStateWasLate_;
}

void LegStateBase::setStateChangedAtTime(double time) {
  stateChangedAtTime_ = time;
}
double LegStateBase::stateChangedAtTime() const {
  return stateChangedAtTime_;
}

double LegStateBase::getPhase() const {
  return phase_;
}

void LegStateBase::setPhase(double phase) {
  phase_ = phase;
}

} /* namespace loco */
