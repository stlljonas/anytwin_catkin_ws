/*
 * EndEffectorBase.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: Dario Bellicoso
 */

#include "loco/common/end_effectors/EndEffectorBase.hpp"

namespace loco {

EndEffectorBase::EndEffectorBase(EndEffectorPropertiesPtr&& endEffectorProperties, bool contactIsAtOrigin)
    : endEffectorStateDesired_(),
      endEffectorStateMeasured_(),
      endEffectorProperties_(std::move(endEffectorProperties)),
      contactIsAtOrigin_(contactIsAtOrigin) {}

void EndEffectorBase::setJointStatesLimb(DesiredJointStates* const desiredJointStatesLimb,
                                         MeasuredJointStates* const measuredJointStatesLimb) {
  for (auto& desiredStateAtTime : endEffectorStateDesired_) {
    for (auto& desiredStateAtFrame : desiredStateAtTime.second) {
      desiredStateAtFrame.second->setDesiredJointStatesLimb(desiredJointStatesLimb);
    }
  }
  for (auto& measuredStateAtTime : endEffectorStateMeasured_) {
    for (auto& measuredStateAtFrame : measuredStateAtTime.second) {
      measuredStateAtFrame.second->setMeasuredJointStatesLimb(measuredJointStatesLimb);
    }
  }
}

const EndEffectorStateDesired& EndEffectorBase::getStateDesired(TimeInstant atTime, EndEffectorFrame atFrame) const {
  if (atFrame == loco::EndEffectorContactEnum::Contact && contactIsAtOrigin_) {
    atFrame = loco::EndEffectorEnum::Origin;
  }
  return *endEffectorStateDesired_.at(atTime).at(atFrame);
}

const EndEffectorStateMeasured& EndEffectorBase::getStateMeasured(TimeInstant atTime, EndEffectorFrame atFrame) const {
  if (atFrame == loco::EndEffectorContactEnum::Contact && contactIsAtOrigin_) {
    atFrame = loco::EndEffectorEnum::Origin;
  }
  return *endEffectorStateMeasured_.at(atTime).at(atFrame);
}

const EndEffectorProperties& EndEffectorBase::getProperties() const {
  return *endEffectorProperties_;
}

EndEffectorStateDesired* EndEffectorBase::getStateDesiredPtr(TimeInstant atTime, EndEffectorFrame atFrame) {
  if (atFrame == loco::EndEffectorContactEnum::Contact && contactIsAtOrigin_) {
    atFrame = loco::EndEffectorEnum::Origin;
  }
  return endEffectorStateDesired_.at(atTime).at(atFrame).get();
}

EndEffectorStateMeasured* EndEffectorBase::getStateMeasuredPtr(TimeInstant atTime, EndEffectorFrame atFrame) {
  if (atFrame == loco::EndEffectorContactEnum::Contact && contactIsAtOrigin_) {
    atFrame = loco::EndEffectorEnum::Origin;
  }
  return endEffectorStateMeasured_.at(atTime).at(atFrame).get();
}

EndEffectorProperties* EndEffectorBase::getPropertiesPtr() {
  return endEffectorProperties_.get();
}

} /* namespace loco */
