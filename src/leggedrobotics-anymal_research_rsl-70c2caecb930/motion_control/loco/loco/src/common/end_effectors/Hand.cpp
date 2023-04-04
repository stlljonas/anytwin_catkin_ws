/*
 * Hand.cpp
 *
 *  Created on: Jun 26, 2018
 *      Author: Markus Staeuble
 */

#include "loco/common/end_effectors/Hand.hpp"

namespace loco {

Hand::Hand(EndEffectorPropertiesPtr&& handProperties, const unsigned int numFingers, bool contactIsAtOrigin)
    : EndEffectorBase(std::move(handProperties), contactIsAtOrigin), numFingers_(numFingers) {
  endEffectorStateDesired_.clear();
  endEffectorStateDesired_[TimePointGait::Now][EndEffectorEnum::Origin].reset(new HandStateDesired(numFingers));
  endEffectorStateDesired_[TimePointGait::LiftOff][EndEffectorEnum::Origin].reset(new HandStateDesired(numFingers));
  endEffectorStateDesired_[TimePointGait::TouchDown][EndEffectorEnum::Origin].reset(new HandStateDesired(numFingers));

  endEffectorStateMeasured_.clear();
  endEffectorStateMeasured_[TimePointGait::Now][EndEffectorEnum::Origin].reset(new HandStateMeasured(numFingers));
  endEffectorStateMeasured_[TimePointGait::LiftOff][EndEffectorEnum::Origin].reset(new HandStateMeasured(numFingers));
  endEffectorStateMeasured_[TimePointGait::TouchDown][EndEffectorEnum::Origin].reset(new HandStateMeasured(numFingers));

  // fingers_.resize(numFingers);
}

const HandStateDesired& Hand::getStateDesired(TimeInstant atTime, EndEffectorFrame atFrame) const {
  return dynamic_cast<const HandStateDesired&>(EndEffectorBase::getStateDesired(atTime, atFrame));
}

const HandStateMeasured& Hand::getStateMeasured(TimeInstant atTime, EndEffectorFrame atFrame) const {
  return dynamic_cast<const HandStateMeasured&>(EndEffectorBase::getStateMeasured(atTime, atFrame));
}

HandStateDesired* Hand::getStateDesiredPtr(TimeInstant atTime, EndEffectorFrame atFrame) {
  return dynamic_cast<HandStateDesired*>(EndEffectorBase::getStateDesiredPtr(atTime, atFrame));
}

HandStateMeasured* Hand::getStateMeasuredPtr(TimeInstant atTime, EndEffectorFrame atFrame) {
  return dynamic_cast<HandStateMeasured*>(EndEffectorBase::getStateMeasuredPtr(atTime, atFrame));
}

} /* namespace loco */
