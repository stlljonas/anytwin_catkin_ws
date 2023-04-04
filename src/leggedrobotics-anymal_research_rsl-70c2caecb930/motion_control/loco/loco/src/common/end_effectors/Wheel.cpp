/*
 * Wheel.cpp
 *
 *  Created on: Dec 21, 2016
 *      Author: Gabriel Hottiger
 */

#include "loco/common/end_effectors/Wheel.hpp"

namespace loco {

Wheel::Wheel(WheelPropertiesPtr&& wheelProperties, const unsigned int indexInLimbJoints, bool contactIsAtOrigin)
    : FootBase(std::move(wheelProperties), contactIsAtOrigin), indexInLimbJoints_(indexInLimbJoints) {
  endEffectorStateDesired_.clear();
  endEffectorStateDesired_[TimePointGait::Now][EndEffectorContactEnum::Origin].reset(new WheelStateDesired(indexInLimbJoints_));
  endEffectorStateDesired_[TimePointGait::LiftOff][EndEffectorContactEnum::Origin].reset(new WheelStateDesired(indexInLimbJoints_));
  endEffectorStateDesired_[TimePointGait::TouchDown][EndEffectorContactEnum::Origin].reset(new WheelStateDesired(indexInLimbJoints_));
  endEffectorStateDesired_[TimePointGait::Now][EndEffectorContactEnum::Contact].reset(new WheelStateDesired(indexInLimbJoints_));
  endEffectorStateDesired_[TimePointGait::LiftOff][EndEffectorContactEnum::Contact].reset(new WheelStateDesired(indexInLimbJoints_));
  endEffectorStateDesired_[TimePointGait::TouchDown][EndEffectorContactEnum::Contact].reset(new WheelStateDesired(indexInLimbJoints_));

  endEffectorStateMeasured_.clear();
  endEffectorStateMeasured_[TimePointGait::Now][EndEffectorContactEnum::Origin].reset(new WheelStateMeasured(indexInLimbJoints_));
  endEffectorStateMeasured_[TimePointGait::LiftOff][EndEffectorContactEnum::Origin].reset(new WheelStateMeasured(indexInLimbJoints_));
  endEffectorStateMeasured_[TimePointGait::TouchDown][EndEffectorContactEnum::Origin].reset(new WheelStateMeasured(indexInLimbJoints_));
  endEffectorStateMeasured_[TimePointGait::Now][EndEffectorContactEnum::Contact].reset(new WheelStateMeasured(indexInLimbJoints_));
  endEffectorStateMeasured_[TimePointGait::LiftOff][EndEffectorContactEnum::Contact].reset(new WheelStateMeasured(indexInLimbJoints_));
  endEffectorStateMeasured_[TimePointGait::TouchDown][EndEffectorContactEnum::Contact].reset(new WheelStateMeasured(indexInLimbJoints_));
}

const WheelStateDesired& Wheel::getStateDesired(TimeInstant atTime, EndEffectorFrame atFrame) const {
  return dynamic_cast<const WheelStateDesired&>(EndEffectorBase::getStateDesired(atTime, atFrame));
}

const WheelStateMeasured& Wheel::getStateMeasured(TimeInstant atTime, EndEffectorFrame atFrame) const {
  return dynamic_cast<const WheelStateMeasured&>(EndEffectorBase::getStateMeasured(atTime, atFrame));
}

const WheelProperties& Wheel::getProperties() const {
  return dynamic_cast<const WheelProperties&>(EndEffectorBase::getProperties());
}

WheelStateDesired* Wheel::getStateDesiredPtr(TimeInstant atTime, EndEffectorFrame atFrame) {
  return dynamic_cast<WheelStateDesired*>(EndEffectorBase::getStateDesiredPtr(atTime, atFrame));
}

WheelStateMeasured* Wheel::getStateMeasuredPtr(TimeInstant atTime, EndEffectorFrame atFrame) {
  return dynamic_cast<WheelStateMeasured*>(EndEffectorBase::getStateMeasuredPtr(atTime, atFrame));
}

WheelProperties* Wheel::getPropertiesPtr() {
  return dynamic_cast<WheelProperties*>(EndEffectorBase::getPropertiesPtr());
}

} /* namespace loco */
