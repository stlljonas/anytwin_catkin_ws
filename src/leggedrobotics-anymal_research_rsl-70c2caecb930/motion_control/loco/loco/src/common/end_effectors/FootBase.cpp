/*
 * FootBase.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: Dario Bellicoso
 */

// loco
#include "loco/common/end_effectors/FootBase.hpp"

namespace loco {

FootBase::FootBase(EndEffectorPropertiesPtr&& endeffectorProperties, bool contactIsAtOrigin)
    : EndEffectorBase(std::move(endeffectorProperties), contactIsAtOrigin) {
  endEffectorStateDesired_.clear();
  endEffectorStateDesired_[TimePointGait::Now][EndEffectorEnum::Origin].reset(new FootBaseStateDesired());
  endEffectorStateDesired_[TimePointGait::LiftOff][EndEffectorEnum::Origin].reset(new FootBaseStateDesired());
  endEffectorStateDesired_[TimePointGait::TouchDown][EndEffectorEnum::Origin].reset(new FootBaseStateDesired());

  endEffectorStateMeasured_.clear();
  endEffectorStateMeasured_[TimePointGait::Now][EndEffectorEnum::Origin].reset(new FootBaseStateMeasured());
  endEffectorStateMeasured_[TimePointGait::LiftOff][EndEffectorEnum::Origin].reset(new FootBaseStateMeasured());
  endEffectorStateMeasured_[TimePointGait::TouchDown][EndEffectorEnum::Origin].reset(new FootBaseStateMeasured());
}

const FootBaseStateDesired& FootBase::getStateDesired(TimeInstant atTime, EndEffectorFrame atFrame) const {
  return dynamic_cast<const FootBaseStateDesired&>(EndEffectorBase::getStateDesired(atTime, atFrame));
}

const FootBaseStateMeasured& FootBase::getStateMeasured(TimeInstant atTime, EndEffectorFrame atFrame) const {
  return dynamic_cast<const FootBaseStateMeasured&>(EndEffectorBase::getStateMeasured(atTime, atFrame));
}

const EndEffectorProperties& FootBase::getProperties() const {
  return EndEffectorBase::getProperties();
}

FootBaseStateDesired* FootBase::getStateDesiredPtr(TimeInstant atTime, EndEffectorFrame atFrame) {
  return dynamic_cast<FootBaseStateDesired*>(EndEffectorBase::getStateDesiredPtr(atTime, atFrame));
}

FootBaseStateMeasured* FootBase::getStateMeasuredPtr(TimeInstant atTime, EndEffectorFrame atFrame) {
  return dynamic_cast<FootBaseStateMeasured*>(EndEffectorBase::getStateMeasuredPtr(atTime, atFrame));
}

EndEffectorProperties* FootBase::getPropertiesPtr() {
  return EndEffectorBase::getPropertiesPtr();
}

} /* namespace loco */
