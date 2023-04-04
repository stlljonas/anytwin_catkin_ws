/*!
 * @file    LegRomo.tpp
 * @author  Gabriel Hottiger
 * @date	  Nov, 2017
 */

#pragma once

// romo_measurements
#include "romo_measurements/legs/LegRomo.hpp"

namespace romo_measurements {

// stateLiftoff_, stateTouchdown_?
template <typename ConcreteDescription_, typename RobotState_, typename LimbBase_>
bool LegRomo<ConcreteDescription_, RobotState_, LimbBase_>::initialize(double dt) {

  if( !this->stateSwitcher_->initialize(dt) ) {
    MELO_WARN_STREAM("[LegRomo]: Leg " << std::string( RD::mapKeyEnumToKeyName(this->limbEnum_) )
                                         << " could not initialize state switcher!");
    return false;
  }

  if( !this->contactSchedule_->initialize(dt) ) {
    MELO_WARN_STREAM("[LegRomo]: Leg " << std::string( RD::mapKeyEnumToKeyName(this->limbEnum_) )
                                       << " could not initialize contact schedule!");
    return false;
  }

  this->didTouchDownAtLeastOnceDuringStance_ = false;
  this->didSetLostContactPositionForPhase_ = false;

  // Advance leg
  if( !LimbRomo::initialize(dt) ) {
    MELO_WARN_STREAM("[LegRomo]: Leg " << std::string( RD::mapKeyEnumToKeyName(this->limbEnum_) )
                                       << " could not initialize LimbRomo!");
    return false;
  }

  // Init last contact positions
  this->positionWorldToLostContactPositionInWorldFrame_ = this->getEndEffector().getStateMeasured(
    loco::TimePoint::Now, loco::EndEffectorContactEnum::Contact).getPositionWorldToEndEffectorInWorldFrame();
  this->positionWorldToLastOrCurrentContactInWorldFrame_ = this->positionWorldToLostContactPositionInWorldFrame_;

  // Initialize lift off and touch down state
  this->stateTouchDown_.setStateChangedAtTime(0.0);
  this->stateTouchDown_.setPositionWorldToFootInWorldFrame(this->getEndEffector().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
  this->stateLiftOff_.setPositionWorldToFootInWorldFrame(this->getEndEffector().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
  this->stateLiftOff_.setPositionWorldToHipInWorldFrame(this->getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame());

  return true;
}

template <typename ConcreteDescription_, typename RobotState_, typename LimbBase_>
bool LegRomo<ConcreteDescription_, RobotState_, LimbBase_>::advance(double dt) {
  if( !LimbRomo::advance(dt) ) {
    MELO_WARN_STREAM("[LegRomo]: Leg " << std::string( RD::mapKeyEnumToKeyName(this->limbEnum_) )
                                       << " could not advance LimbRomo!");
    return false;
  }

  // Update contact and slipping state.
  this->getContactSchedulePtr()->setWasGrounded(this->getContactSchedule().isGrounded());
  this->getContactSchedulePtr()->setIsGrounded(this->getFoot().isInContact());
  this->getContactSchedulePtr()->setIsSlipping(this->getFoot().isSlipping());

  // Update the position of the most recent contact location.
  if (this->getContactSchedule().isGrounded() && !this->getContactSchedule().isSlipping()) {
    this->positionWorldToLastOrCurrentContactInWorldFrame_ = this->getEndEffector().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
  }

  return true;
}

}  // namespace romo_measurements
